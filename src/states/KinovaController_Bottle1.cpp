#include "KinovaController_Bottle1.h"

#include "../KinovaController.h"

#include <RBDyn/FK.h>

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <map>

static const std::vector<std::string> jointNames = {"joint_1", "joint_2", "joint_3", "joint_4",
                                                    "joint_5", "joint_6", "joint_7"};

void KinovaController_Bottle1::configure(const mc_rtc::Configuration & config)
{
  config("poseApproach_deg", poseApproachDeg_);
  config("poseA_deg", poseADeg_);
  config("poseLift_deg", poseLiftDeg_);
  config("poseB_deg", poseBDeg_);
  config("approachPos", approachPos_);
  config("graspPos", graspPos_);
  config("liftPos", liftPos_);
  config("lateralOffset", lateralOffset_);
  config("gripperClosedPct", gripperClosedPct_);
  config("approachSpeed", approachSpeed_);
  config("graspSpeed", graspSpeed_);
  config("liftSpeed", liftSpeed_);
  config("presentSpeed", presentSpeed_);
  config("returnSpeed", returnSpeed_);
  config("pullJoint4Thresh", pullJoint4Thresh_);
  config("pullJoint2Thresh", pullJoint2Thresh_);
  config("pullJoint4Abs", pullJoint4Abs_);
  config("pullJoint2Abs", pullJoint2Abs_);
  config("earlyPullJoint4Abs", earlyPullJoint4Abs_);
  config("earlyPullJoint2Abs", earlyPullJoint2Abs_);
  config("earlyDebounceTime", earlyDebounceTime_);
  config("armDelay", armDelay_);
  config("debounceTime", debounceTime_);
  config("baselineWindow", baselineWindow_);
  config("gripperSettle", gripperSettle_);
  config("convergenceGrace", convergenceGrace_);
  config("minMoveTime", minMoveTime_);
  config("autoReleaseAfter", autoReleaseAfter_);
  // Per-run CSV log (see header). Default directory: $HOME/workspace/logs/runs
  const char * home = std::getenv("HOME");
  runLogDir_ = std::string(home ? home : "/tmp") + "/workspace/logs/runs";
  config("runLogEnabled", runLogEnabled_);
  config("runLogDir", runLogDir_);
  config("runLogDecimation", runLogDecimation_);
  config("runLogReserveSec", runLogReserveSec_);
}

void KinovaController_Bottle1::note(const std::string & text)
{
  mc_rtc::log::info("[{}] {}", name(), text);
  runLog_.event(runTime_, text);
}

sva::PTransformd KinovaController_Bottle1::poseFK(mc_control::fsm::Controller & ctl_,
                                                  const std::vector<double> & poseDeg) const
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  auto & robot = ctl.robot();
  // Work on a copy of the live config so the actual robot state is untouched.
  rbd::MultiBodyConfig mbcTmp = robot.mbc();
  for(size_t i = 0; i < jointNames.size(); ++i)
  {
    mbcTmp.q[robot.jointIndexByName(jointNames[i])][0] = mc_rtc::constants::toRad(poseDeg[i]);
  }
  rbd::forwardKinematics(robot.mb(), mbcTmp);
  return mbcTmp.bodyPosW[robot.bodyIndexByName("end_effector_link")];
}

Eigen::Vector3d KinovaController_Bottle1::resolvePoint(const std::vector<double> & override3,
                                                       const sva::PTransformd & fk)
{
  if(override3.size() == 3)
  {
    return Eigen::Vector3d(override3[0], override3[1], override3[2]);
  }
  return fk.translation();
}

void KinovaController_Bottle1::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);

  // Validate config so a bad YAML can never send the arm somewhere unsafe.
  for(const auto * p : {&poseApproachDeg_, &poseADeg_, &poseLiftDeg_, &poseBDeg_})
  {
    if(p->size() != jointNames.size())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[Bottle1] every pose_deg must have {} entries",
                                                       jointNames.size());
    }
  }
  for(const auto * o : {&approachPos_, &graspPos_, &liftPos_})
  {
    if(!o->empty() && o->size() != 3)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[Bottle1] position overrides must have 3 entries (got {})",
                                                       o->size());
    }
  }
  for(double speed : {approachSpeed_, graspSpeed_, liftSpeed_, presentSpeed_, returnSpeed_})
  {
    if(!(speed > 0.0))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[Bottle1] all speeds must be > 0 (got {})", speed);
    }
  }
  if(!(gripperClosedPct_ >= 0.0 && gripperClosedPct_ <= 100.0))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[Bottle1] gripperClosedPct must be in [0, 100] (got {})",
                                                     gripperClosedPct_);
  }

  // Adopt the active handover condition (motion style) for the whole sequence.
  alpha_ = ctl.handoverAlpha();
  durationFactor_ = ctl.handoverDurationFactor();
  speedScale_ = ctl.handoverSpeedScale();
  armDelay_ = ctl.handoverArmDelay();
  mc_rtc::log::info("[{}] Handover condition: {} (speed x{:.3f}, alpha {:.2f}, armDelay {:.2f}s)", name(),
                    ctl.handoverConditionName(), speedScale_, alpha_, armDelay_);

  // Measure the pick from the taught poses via forward kinematics; config
  // overrides (metres) take precedence. The grasp orientation (top-down) is
  // always the FK orientation of poseA and is held over approach/grasp/lift.
  const sva::PTransformd fkApproach = poseFK(ctl_, poseApproachDeg_);
  const sva::PTransformd fkGrasp = poseFK(ctl_, poseADeg_);
  const sva::PTransformd fkLift = poseFK(ctl_, poseLiftDeg_);
  approachPt_ = resolvePoint(approachPos_, fkApproach);
  graspPt_ = resolvePoint(graspPos_, fkGrasp);
  liftPt_ = resolvePoint(liftPos_, fkLift);
  // Shift the whole column laterally (+y = left) so Bottle1..4 line up in a row
  // with a common x, z and grasp orientation, differing only in y.
  const Eigen::Vector3d lateral(0.0, lateralOffset_, 0.0);
  approachPt_ += lateral;
  graspPt_ += lateral;
  liftPt_ += lateral;
  graspOri_ = fkGrasp.rotation();
  // Present-pose EE position (FK of poseB), for the "distance still to reach"
  // diagnostic logged in every phase. poseB is joint-space and common to all
  // bottles, so this is the same point regardless of lateralOffset.
  presentEEPos_ = poseFK(ctl_, poseBDeg_).translation();
  mc_rtc::log::info("[{}] approach [{:.3f}, {:.3f}, {:.3f}] grasp [{:.3f}, {:.3f}, {:.3f}] lift [{:.3f}, {:.3f}, "
                    "{:.3f}] (m)",
                    name(), approachPt_.x(), approachPt_.y(), approachPt_.z(), graspPt_.x(), graspPt_.y(), graspPt_.z(),
                    liftPt_.x(), liftPt_.y(), liftPt_.z());

  // --- Per-run CSV log: one file per execution of this state --------------------
  runTime_ = 0.0;
  tick_ = 0;
  releaseTrigger_ = 0;
  segProgress_ = 0.0;
  jointIdx_.clear();
  dofIdx_.clear();
  refTorqueIdx_.clear();
  const auto & rjo = ctl.robot().refJointOrder();
  for(const auto & jn : jointNames)
  {
    jointIdx_.push_back(ctl.robot().jointIndexByName(jn));
    dofIdx_.push_back(ctl.robot().mb().jointPosInDof(jointIdx_.back()));
    auto it = std::find(rjo.begin(), rjo.end(), jn);
    refTorqueIdx_.push_back(it == rjo.end() ? -1 : static_cast<int>(it - rjo.begin()));
  }
  if(runLogEnabled_)
  {
    std::vector<std::string> cols = {"t", "phase"};
    auto addJointCols = [&cols](const std::string & prefix)
    {
      for(size_t i = 1; i <= jointNames.size(); ++i)
      {
        cols.push_back(prefix + std::to_string(i));
      }
    };
    addJointCols("q_"); // measured joint angle [rad] (realRobot / encoders)
    addJointCols("qd_"); // measured joint velocity [rad/s]
    addJointCols("q_cmd_"); // commanded joint angle [rad] (control robot)
    addJointCols("tau_meas_"); // measured joint torque [Nm] (realRobot().jointTorques())
    addJointCols("tau_ext_"); // external-torque estimate [Nm] (ExternalForcesEstimator)
    for(const char * c : {"ee_x",
                          "ee_y",
                          "ee_z",
                          "ee_vx",
                          "ee_vy",
                          "ee_vz",
                          "ee_cmd_x",
                          "ee_cmd_y",
                          "ee_cmd_z",
                          "ee_ref_x",
                          "ee_ref_y",
                          "ee_ref_z",
                          "dist_to_present",
                          "seg_progress",
                          "hold_timer",
                          "baseline_captured",
                          "d2",
                          "d4",
                          "debounce_timer",
                          "early_debounce_timer",
                          "gripper_opening",
                          "release_trigger"})
    {
      cols.push_back(c);
    }
    const int decim = std::max(runLogDecimation_, 1);
    const size_t reserveRows = static_cast<size_t>(std::max(runLogReserveSec_, 1.0) / (ctl.timeStep * decim)) + 1;
    if(runLog_.begin(runLogDir_, name() + "_" + ctl.handoverConditionName(), cols, reserveRows))
    {
      // Wall-clock start with millisecond precision, so a run can be aligned
      // with the heart-rate samples and the video clock (both wall-clock).
      // t=0 of the rows below is this instant (within one control tick).
      const auto nowTp = std::chrono::system_clock::now();
      const long long epochMs = std::chrono::duration_cast<std::chrono::milliseconds>(nowTp.time_since_epoch()).count();
      const std::time_t now = std::chrono::system_clock::to_time_t(nowTp);
      std::tm tmLocal;
      ::localtime_r(&now, &tmLocal);
      char whenSec[32];
      char tz[8];
      std::strftime(whenSec, sizeof(whenSec), "%Y-%m-%d %H:%M:%S", &tmLocal);
      std::strftime(tz, sizeof(tz), "%z", &tmLocal);
      const std::string when = fmt::format("{}.{:03d} {}", whenSec, static_cast<int>(epochMs % 1000), tz);
      runLog_.meta(fmt::format("state: {}", name()));
      runLog_.meta(fmt::format("started: {}", when));
      runLog_.meta(fmt::format("started_epoch_ms: {}", epochMs));
      const long long voiceMs = ctl.consumeVoiceEpochMs();
      if(voiceMs > 0)
      {
        runLog_.meta(fmt::format("voice_request_epoch_ms: {} (start {:.3f} s after the request, voice {:.1f} dBFS)",
                                 voiceMs, (epochMs - voiceMs) / 1000.0, ctl.pendingVoiceLevelDb_));
      }
      else
      {
        runLog_.meta("voice_request_epoch_ms: none (not voice-triggered)");
      }
      runLog_.meta(fmt::format("robot: {}", ctl.robot().name()));
      runLog_.meta(fmt::format("timestep: {}", ctl.timeStep));
      runLog_.meta(fmt::format("decimation: {}", decim));
      runLog_.meta(fmt::format("session_log: {}", ctl.logger().path()));
      runLog_.meta(fmt::format("condition: {}", ctl.handoverConditionName()));
      runLog_.meta(fmt::format("speed_scale: {}", speedScale_));
      runLog_.meta(fmt::format("alpha: {}", alpha_));
      runLog_.meta(fmt::format("arm_delay: {}", armDelay_));
      runLog_.meta(fmt::format("duration_factor: {} (move duration = factor * distance / (speed * speed_scale); "
                               "same for every condition)",
                               durationFactor_));
      runLog_.meta(fmt::format("lateral_offset: {}", lateralOffset_));
      runLog_.meta(fmt::format("approach_pt: {} {} {}", approachPt_.x(), approachPt_.y(), approachPt_.z()));
      runLog_.meta(fmt::format("grasp_pt: {} {} {}", graspPt_.x(), graspPt_.y(), graspPt_.z()));
      runLog_.meta(fmt::format("lift_pt: {} {} {}", liftPt_.x(), liftPt_.y(), liftPt_.z()));
      runLog_.meta(fmt::format("present_ee: {} {} {}", presentEEPos_.x(), presentEEPos_.y(), presentEEPos_.z()));
      runLog_.meta(fmt::format("speeds: approach {} grasp {} lift {} (m/s) present {} return {} (rad/s)",
                               approachSpeed_, graspSpeed_, liftSpeed_, presentSpeed_, returnSpeed_));
      runLog_.meta(fmt::format("gripper_closed_pct: {}", gripperClosedPct_));
      runLog_.meta(fmt::format("pull_thresholds: d4>{} |d2|>{} abs4>{} |abs2|>{} early4>{} |early2|>{} "
                               "debounce {} early_debounce {}",
                               pullJoint4Thresh_, pullJoint2Thresh_, pullJoint4Abs_, pullJoint2Abs_,
                               earlyPullJoint4Abs_, earlyPullJoint2Abs_, debounceTime_, earlyDebounceTime_));
      runLog_.meta(fmt::format("auto_release_after: {}", autoReleaseAfter_));
      runLog_.meta(
          fmt::format("baseline_window: {} (pull-detection baseline = mean over this many seconds)", baselineWindow_));
      runLog_.meta("phases: 0 MOVE_TO_APPROACH, 1 MOVE_TO_GRASP, 2 CLOSE_GRIPPER, 3 MOVE_TO_LIFTUP, "
                   "4 MOVE_TO_PRESENT, 5 HOLDING, 6 RELEASING, 7 RETURN_TO_HOME, 8 DONE");
      runLog_.meta("release_trigger: 0 none, 1 pull (delta/abs), 2 early pull, 3 manual button, 4 auto (sim)");
      runLog_.meta("units: q/q_cmd rad, qd rad/s, tau Nm, ee m and m/s, timers s; q/qd/tau_meas/ee from "
                   "realRobot (encoders), q_cmd/ee_cmd from the control robot, ee_ref = EE task reference "
                   "(meaningful in Cartesian phases 0-3 only), events/meta lines start with '#'");
      rowBuf_.reserve(cols.size());
      mc_rtc::log::info("[{}] Run log -> {}", name(), runLog_.path());
    }
  }

  // Open the gripper before the pick (as BottlePick does).
  ctl.robot().gripper("gripper").setTargetOpening(1.0);

  // Operator-triggered safe return (same manual fallback as BottlePick).
  returnRequested_ = false;
  ctl.gui()->addElement({"Bottle1"}, mc_rtc::gui::Button("Return home (safe)", [this]() { returnRequested_ = true; }));

  // Pull-detection wiring (identical to BottlePick).
  // tau2/tau4 are the external-torque estimate on joint_2/joint_4 — now updated in
  // EVERY phase (see run()), so the whole approach/grasp/lift/present motion is
  // captured, not just HOLDING. d2/d4 are the deviation from the resting baseline
  // and are only meaningful during HOLDING (0 before the baseline is armed).
  ctl.logger().addLogEntry("Bottle1_tau2", [this]() { return lastTau2_; });
  ctl.logger().addLogEntry("Bottle1_tau4", [this]() { return lastTau4_; });
  ctl.logger().addLogEntry("Bottle1_d2", [this]() { return lastD2_; });
  ctl.logger().addLogEntry("Bottle1_d4", [this]() { return lastD4_; });
  // Phase index (see enum Phase) and the live EE distance to the present pose, so
  // the external-torque trace can be split by phase and correlated with "how far
  // the arm still had to reach" when a pull came in (basis for case-A thresholds).
  ctl.logger().addLogEntry("Bottle1_phase", [this]() { return static_cast<int>(phase_); });
  ctl.logger().addLogEntry("Bottle1_eeDistToPresent", [this]() { return lastDistToPresent_; });
  dofJoint2_ = ctl.robot().mb().jointPosInDof(ctl.robot().jointIndexByName("joint_2"));
  dofJoint4_ = ctl.robot().mb().jointPosInDof(ctl.robot().jointIndexByName("joint_4"));
  forceDetectAvailable_ = ctl.datastore().has("EF_Estimator::getExternalTorques");
  if(!forceDetectAvailable_)
  {
    mc_rtc::log::warning("[Bottle1] ExternalForcesEstimator read-call not found; "
                         "pull-to-release disabled, use the Return home button");
  }

  // Self-contained EE-task management (as NGPoint): Initial leaves compEETask out
  // of the solver, so remove-then-reset targets the CURRENT pose (no jump), gear
  // it, and add it back. The grasp orientation is a step input; the 100/50 gains
  // rotate the wrist there smoothly during the approach.
  ctl.solver().removeTask(ctl.compEETask);
  ctl.compEETask->reset();
  ctl.compEETask->positionTask->refVel(Eigen::Vector3d::Zero());
  ctl.compEETask->positionTask->refAccel(Eigen::Vector3d::Zero());
  ctl.compEETask->positionTask->stiffness(100.0);
  ctl.compEETask->positionTask->damping(50.0);
  ctl.compEETask->orientationTask->stiffness(100.0);
  ctl.compEETask->orientationTask->damping(50.0);
  ctl.compEETask->orientationTask->orientation(graspOri_);
  ctl.solver().addTask(ctl.compEETask);
  // Low-weight posture regularization anchored on home (null-space bias).
  ctl.compPostureTask->target(ctl.postureHome);

  initSegment(ctl_, approachPt_, approachSpeed_, /*continuous=*/false);
  phase_ = MOVE_TO_APPROACH;
  note(fmt::format("Moving to approach point ({:.1f} s)", tf_));
}

// --- Cartesian segment (EE task, blended s_alpha; same as NGPoint) -----------

void KinovaController_Bottle1::initSegment(mc_control::fsm::Controller & ctl_,
                                           const Eigen::Vector3d & goal,
                                           double speed,
                                           bool continuous)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  segStart_ = continuous ? segEnd_ : Eigen::Vector3d(ctl.compEETask->positionTask->position());
  segEnd_ = goal;
  const double dist = (segEnd_ - segStart_).norm();
  // Same duration in every condition (alpha only shapes the profile).
  const double peakFactor = durationFactor_;
  tf_ = std::max(peakFactor * dist / (speed * speedScale_), minMoveTime_);
  t_ = 0.0;
}

bool KinovaController_Bottle1::segmentDone(mc_control::fsm::Controller & ctl_, bool requireConvergence)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  t_ += ctl.timeStep;
  const double u = std::min(t_ / tf_, 1.0);
  const Eigen::Vector3d delta = segEnd_ - segStart_;
  if(t_ < tf_)
  {
    const double mj = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    const double s = (1.0 - alpha_) * u + alpha_ * mj;
    const double dsdu = (1.0 - alpha_) + alpha_ * 30.0 * u * u * (1.0 - u) * (1.0 - u);
    const double d2sdu2 = alpha_ * (60.0 * u - 180.0 * u * u + 120.0 * u * u * u);
    ctl.compEETask->positionTask->position(segStart_ + s * delta);
    ctl.compEETask->positionTask->refVel(delta * (dsdu / tf_));
    ctl.compEETask->positionTask->refAccel(delta * (d2sdu2 / (tf_ * tf_)));
    return false;
  }
  ctl.compEETask->positionTask->position(segEnd_);
  ctl.compEETask->positionTask->refVel(Eigen::Vector3d::Zero());
  ctl.compEETask->positionTask->refAccel(Eigen::Vector3d::Zero());
  if(!requireConvergence)
  {
    return true;
  }
  if(ctl.compEETask->eval().norm() < 0.01)
  {
    return true;
  }
  if(t_ > tf_ + convergenceGrace_)
  {
    mc_rtc::log::warning("[Bottle1] Segment did not fully converge (task error {:.3f}), continuing",
                         ctl.compEETask->eval().norm());
    return true;
  }
  return false;
}

// --- Joint-space move (posture task, blended; same as BottlePick) ------------

void KinovaController_Bottle1::initMove(mc_control::fsm::Controller & ctl_,
                                        const std::vector<double> & poseDeg,
                                        double speed,
                                        bool continuous)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  constexpr double TWO_PI = 2.0 * mc_rtc::constants::PI;
  std::vector<double> prevTarget = targetQ_;
  startQ_.clear();
  targetQ_.clear();
  double maxDelta = 0.0;
  for(size_t i = 0; i < jointNames.size(); ++i)
  {
    double ref = (continuous && i < prevTarget.size())
                     ? prevTarget[i]
                     : ctl.robot().mbc().q[ctl.robot().jointIndexByName(jointNames[i])][0];
    double target = mc_rtc::constants::toRad(poseDeg[i]);
    target += TWO_PI * std::round((ref - target) / TWO_PI); // folding: nearest equivalent angle
    startQ_.push_back(ref);
    targetQ_.push_back(target);
    maxDelta = std::max(maxDelta, std::abs(target - ref));
  }
  // Same duration in every condition (alpha only shapes the profile).
  const double peakFactor = durationFactor_;
  tf_ = std::max(peakFactor * maxDelta / (speed * speedScale_), minMoveTime_);
  t_ = 0.0;
}

bool KinovaController_Bottle1::moveDone(mc_control::fsm::Controller & ctl_, bool requireConvergence)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  t_ += ctl.timeStep;
  double u = std::min(t_ / tf_, 1.0);
  double mj = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
  double s = (1.0 - alpha_) * u + alpha_ * mj;
  std::map<std::string, std::vector<double>> target;
  double maxErr = 0.0;
  for(size_t i = 0; i < jointNames.size(); ++i)
  {
    target[jointNames[i]] = {startQ_[i] + s * (targetQ_[i] - startQ_[i])};
    double q = ctl.robot().mbc().q[ctl.robot().jointIndexByName(jointNames[i])][0];
    maxErr = std::max(maxErr, std::abs(q - targetQ_[i]));
  }
  ctl.compPostureTask->target(target);
  if(t_ < tf_)
  {
    return false;
  }
  if(!requireConvergence)
  {
    return true;
  }
  if(maxErr < 0.05)
  {
    return true;
  }
  if(t_ > tf_ + convergenceGrace_)
  {
    mc_rtc::log::warning("[Bottle1] Move did not fully converge (max joint error {:.3f} rad), continuing", maxErr);
    return true;
  }
  return false;
}

bool KinovaController_Bottle1::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  // Sample the external-torque estimate and the EE distance to the present pose
  // EVERY tick, in every phase, so the logged trace covers the whole motion (the
  // diagnostic for releasing before the arm is fully extended). HOLDING reuses
  // these samples below instead of reading the datastore a second time.
  if(forceDetectAvailable_)
  {
    tauExt_ = ctl.datastore().call<Eigen::VectorXd>("EF_Estimator::getExternalTorques");
    lastTau2_ = tauExt_[dofJoint2_];
    lastTau4_ = tauExt_[dofJoint4_];
  }
  // Physical (encoder-based) EE distance to the present pose.
  lastDistToPresent_ = (ctl.realRobot().bodyPosW("end_effector_link").translation() - presentEEPos_).norm();
  runTime_ += ctl.timeStep;
  segProgress_ = (tf_ > 0.0) ? std::min(t_ / tf_, 1.0) : 0.0;
  releaseTrigger_ = 0;
  switch(phase_)
  {
    case MOVE_TO_APPROACH:
      // Via-point: flow straight into the descent
      if(segmentDone(ctl_, /*requireConvergence=*/false))
      {
        initSegment(ctl_, graspPt_, graspSpeed_, /*continuous=*/true);
        phase_ = MOVE_TO_GRASP;
        note(fmt::format("Descending to grasp point ({:.1f} s)", tf_));
      }
      break;
    case MOVE_TO_GRASP:
      // Grasp must be accurate: wait for convergence before closing.
      if(segmentDone(ctl_, /*requireConvergence=*/true))
      {
        note(fmt::format("Grasp point reached, closing gripper to {:.0f}%", gripperClosedPct_));
        ctl.robot().gripper("gripper").setTargetOpening(1.0 - gripperClosedPct_ / 100.0);
        timer_ = 0.0;
        phase_ = CLOSE_GRIPPER;
      }
      break;
    case CLOSE_GRIPPER:
      // Arm holds the grasp point (EE target unchanged) while the gripper closes.
      timer_ += ctl.timeStep;
      if((timer_ > gripperSettle_ && ctl.robot().gripper("gripper").complete()) || timer_ > 4.0)
      {
        if(timer_ > 4.0)
        {
          mc_rtc::log::warning("[{}] Gripper close did not report completion, continuing anyway", name());
        }
        initSegment(ctl_, liftPt_, liftSpeed_, /*continuous=*/true);
        phase_ = MOVE_TO_LIFTUP;
        note(fmt::format("Gripper closed, lifting straight up ({:.1f} s)", tf_));
      }
      break;
    case MOVE_TO_LIFTUP:
      // Settle at the lift point (converge) before switching Cartesian -> joint,
      // so the EE velocity is ~0 when the EE task is dropped (no lurch).
      if(segmentDone(ctl_, /*requireConvergence=*/true))
      {
        // Hand off to joint space for the present pose: drop the EE task so the
        // posture task drives the joints, starting from the current q (no jump).
        ctl.solver().removeTask(ctl.compEETask);
        initMove(ctl_, poseBDeg_, presentSpeed_, /*continuous=*/false);
        phase_ = MOVE_TO_PRESENT;
        note(fmt::format("Lifted, moving to present pose ({:.1f} s)", tf_));
      }
      break;
    case MOVE_TO_PRESENT:
      if(moveDone(ctl_, /*requireConvergence=*/true))
      {
        note("Present pose reached, holding. Pull the bottle to release, or press \"Return home (safe)\".");
        holdTimer_ = 0.0;
        debounceTimer_ = 0.0;
        earlyDebounceTimer_ = 0.0;
        baselineCaptured_ = false;
        baselineSum2_ = baselineSum4_ = 0.0;
        baselineN_ = 0;
        baselineTimer_ = 0.0;
        phase_ = HOLDING;
      }
      break;
    case HOLDING:
    {
      holdTimer_ += ctl.timeStep;
      bool release = returnRequested_;
      if(release)
      {
        releaseTrigger_ = 3;
        note("Manual release requested (button)");
      }
      else if(autoReleaseAfter_ > 0.0 && holdTimer_ > autoReleaseAfter_)
      {
        release = true;
        releaseTrigger_ = 4;
        note(fmt::format("Auto release after {:.2f} s of holding (simulation aid)", holdTimer_));
      }
      if(!release && forceDetectAvailable_)
      {
        // Reuse this tick's samples taken at the top of run().
        double t2 = lastTau2_;
        double t4 = lastTau4_;
        bool over = false;
        bool earlyOver = (t4 > earlyPullJoint4Abs_) || (t2 > earlyPullJoint2Abs_) || (t2 < -earlyPullJoint2Abs_);
        if(holdTimer_ > armDelay_)
        {
          if(!baselineCaptured_)
          {
            // Average over baselineWindow_ (no delta detection meanwhile; the
            // absolute-threshold early path below stays active).
            baselineSum2_ += t2;
            baselineSum4_ += t4;
            ++baselineN_;
            baselineTimer_ += ctl.timeStep;
            if(baselineTimer_ >= baselineWindow_)
            {
              baselineJoint2_ = baselineSum2_ / baselineN_;
              baselineJoint4_ = baselineSum4_ / baselineN_;
              baselineCaptured_ = true;
              lastD2_ = 0.0;
              lastD4_ = 0.0;
              note(fmt::format("Detection armed, baseline joint_2={:.2f} joint_4={:.2f} (mean of {:.2f} s)",
                               baselineJoint2_, baselineJoint4_, baselineTimer_));
            }
          }
          else
          {
            double d2 = t2 - baselineJoint2_;
            double d4 = t4 - baselineJoint4_;
            lastD2_ = d2;
            lastD4_ = d4;
            bool deltaOver = (d4 > pullJoint4Thresh_) || (d2 > pullJoint2Thresh_) || (d2 < -pullJoint2Thresh_);
            bool absOver = (t4 > pullJoint4Abs_) || (t2 > pullJoint2Abs_) || (t2 < -pullJoint2Abs_);
            over = deltaOver || absOver;
          }
        }
        debounceTimer_ = over ? debounceTimer_ + ctl.timeStep : 0.0;
        earlyDebounceTimer_ = earlyOver ? earlyDebounceTimer_ + ctl.timeStep : 0.0;
        if(debounceTimer_ > debounceTime_ || earlyDebounceTimer_ > earlyDebounceTime_)
        {
          bool viaEarly = !(debounceTimer_ > debounceTime_);
          note(fmt::format("Pull detected (d_joint_2={:.2f}, d_joint_4={:.2f}, abs_joint_2={:.2f}, "
                           "abs_joint_4={:.2f}, early={}), releasing",
                           lastD2_, lastD4_, t2, t4, viaEarly));
          release = true;
          releaseTrigger_ = viaEarly ? 2 : 1;
        }
      }
      if(release)
      {
        ctl.robot().gripper("gripper").setTargetOpening(1.0);
        timer_ = 0.0;
        phase_ = RELEASING;
      }
      break;
    }
    case RELEASING:
      timer_ += ctl.timeStep;
      if((timer_ > gripperSettle_ && ctl.robot().gripper("gripper").complete()) || timer_ > 2.0)
      {
        std::vector<double> homeDeg;
        for(const auto & jn : jointNames)
        {
          homeDeg.push_back(ctl.postureHome.at(jn)[0] * 180.0 / mc_rtc::constants::PI);
        }
        initMove(ctl_, homeDeg, returnSpeed_, /*continuous=*/true);
        note(fmt::format("Bottle released, moving straight to home ({:.1f} s)", tf_));
        phase_ = RETURN_TO_HOME;
      }
      break;
    case RETURN_TO_HOME:
      if(moveDone(ctl_, /*requireConvergence=*/true))
      {
        note("Returned home safely.");
        phase_ = DONE;
      }
      break;
    case DONE:
      logRow(ctl_);
      output("OK");
      return true;
  }
  logRow(ctl_);
  return false;
}

void KinovaController_Bottle1::logRow(mc_control::fsm::Controller & ctl_)
{
  if(!runLog_.active())
  {
    return;
  }
  if((tick_++ % std::max(runLogDecimation_, 1)) != 0)
  {
    return;
  }
  auto & ctl = static_cast<KinovaController &>(ctl_);
  const auto & real = ctl.realRobot();
  const auto & cmd = ctl.robot();
  const auto & tauMeas = real.jointTorques();
  rowBuf_.clear();
  rowBuf_.push_back(runTime_);
  rowBuf_.push_back(static_cast<double>(phase_));
  for(int j : jointIdx_)
  {
    rowBuf_.push_back(real.mbc().q[j][0]);
  }
  for(int j : jointIdx_)
  {
    rowBuf_.push_back(real.mbc().alpha[j][0]);
  }
  for(int j : jointIdx_)
  {
    rowBuf_.push_back(cmd.mbc().q[j][0]);
  }
  for(int r : refTorqueIdx_)
  {
    rowBuf_.push_back((r >= 0 && static_cast<size_t>(r) < tauMeas.size()) ? tauMeas[r] : 0.0);
  }
  for(int d : dofIdx_)
  {
    rowBuf_.push_back(d < tauExt_.size() ? tauExt_[d] : 0.0);
  }
  const Eigen::Vector3d eePos = real.bodyPosW("end_effector_link").translation();
  const Eigen::Vector3d eeVel = real.bodyVelW("end_effector_link").linear();
  const Eigen::Vector3d eeCmd = cmd.bodyPosW("end_effector_link").translation();
  const Eigen::Vector3d eeRef = ctl.compEETask->positionTask->position();
  for(const Eigen::Vector3d * v : {&eePos, &eeVel, &eeCmd, &eeRef})
  {
    rowBuf_.push_back(v->x());
    rowBuf_.push_back(v->y());
    rowBuf_.push_back(v->z());
  }
  rowBuf_.push_back(lastDistToPresent_);
  rowBuf_.push_back(segProgress_);
  rowBuf_.push_back(phase_ == HOLDING ? holdTimer_ : 0.0);
  rowBuf_.push_back(baselineCaptured_ ? 1.0 : 0.0);
  rowBuf_.push_back(lastD2_);
  rowBuf_.push_back(lastD4_);
  rowBuf_.push_back(debounceTimer_);
  rowBuf_.push_back(earlyDebounceTimer_);
  rowBuf_.push_back(cmd.gripper("gripper").opening());
  rowBuf_.push_back(static_cast<double>(releaseTrigger_));
  runLog_.row(rowBuf_);
}

void KinovaController_Bottle1::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  // Leave the solver as Initial expects (no EE task); already removed at the
  // lift->present handoff, so this is a harmless no-op on the normal path.
  ctl.solver().removeTask(ctl.compEETask);
  ctl.gui()->removeElement({"Bottle1"}, "Return home (safe)");
  ctl.logger().removeLogEntry("Bottle1_tau2");
  ctl.logger().removeLogEntry("Bottle1_tau4");
  ctl.logger().removeLogEntry("Bottle1_d2");
  ctl.logger().removeLogEntry("Bottle1_d4");
  ctl.logger().removeLogEntry("Bottle1_phase");
  ctl.logger().removeLogEntry("Bottle1_eeDistToPresent");
  // Hand the buffered run to the background writer (no disk I/O on this thread).
  if(runLog_.active())
  {
    if(phase_ != DONE)
    {
      runLog_.event(runTime_, "Interrupted before completion (emergency stop or forced transition)");
    }
    mc_rtc::log::info("[{}] Run finished after {:.2f} s, {} samples -> {}", name(), runTime_, runLog_.rows(),
                      runLog_.path());
    runLog_.finish();
  }
}

EXPORT_SINGLE_STATE("KinovaController_Bottle1", KinovaController_Bottle1)
