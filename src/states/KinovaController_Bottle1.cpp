#include "KinovaController_Bottle1.h"

#include "../KinovaController.h"

#include <RBDyn/FK.h>

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>

#include <Eigen/Core>

#include <cmath>
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
  config("gripperSettle", gripperSettle_);
  config("convergenceGrace", convergenceGrace_);
  config("minMoveTime", minMoveTime_);
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
  speedScale_ = ctl.handoverSpeedScale();
  armDelay_ = ctl.handoverArmDelay();
  mc_rtc::log::info("[Bottle1] Handover condition: {} (speed x{:.3f}, alpha {:.2f}, armDelay {:.2f}s)",
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
  mc_rtc::log::info("[Bottle1] approach [{:.3f}, {:.3f}, {:.3f}] grasp [{:.3f}, {:.3f}, {:.3f}] lift [{:.3f}, {:.3f}, "
                    "{:.3f}] (m)",
                    approachPt_.x(), approachPt_.y(), approachPt_.z(), graspPt_.x(), graspPt_.y(), graspPt_.z(),
                    liftPt_.x(), liftPt_.y(), liftPt_.z());

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
  mc_rtc::log::info("[Bottle1] Moving to approach point ({:.1f} s)", tf_);
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
  const double peakFactor = 1.0 + 0.875 * alpha_;
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
  const double peakFactor = 1.0 + 0.875 * alpha_;
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
    const Eigen::VectorXd tauExt = ctl.datastore().call<Eigen::VectorXd>("EF_Estimator::getExternalTorques");
    lastTau2_ = tauExt[dofJoint2_];
    lastTau4_ = tauExt[dofJoint4_];
  }
  lastDistToPresent_ = (ctl.robot().bodyPosW("end_effector_link").translation() - presentEEPos_).norm();
  switch(phase_)
  {
    case MOVE_TO_APPROACH:
      // Via-point: flow straight into the descent
      if(segmentDone(ctl_, /*requireConvergence=*/false))
      {
        initSegment(ctl_, graspPt_, graspSpeed_, /*continuous=*/true);
        phase_ = MOVE_TO_GRASP;
        mc_rtc::log::info("[Bottle1] Descending to grasp point ({:.1f} s)", tf_);
      }
      break;
    case MOVE_TO_GRASP:
      // Grasp must be accurate: wait for convergence before closing.
      if(segmentDone(ctl_, /*requireConvergence=*/true))
      {
        mc_rtc::log::info("[Bottle1] Grasp point reached, closing gripper to {:.0f}%", gripperClosedPct_);
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
          mc_rtc::log::warning("[Bottle1] Gripper close did not report completion, continuing anyway");
        }
        initSegment(ctl_, liftPt_, liftSpeed_, /*continuous=*/true);
        phase_ = MOVE_TO_LIFTUP;
        mc_rtc::log::info("[Bottle1] Gripper closed, lifting straight up ({:.1f} s)", tf_);
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
        mc_rtc::log::info("[Bottle1] Lifted, moving to present pose ({:.1f} s)", tf_);
      }
      break;
    case MOVE_TO_PRESENT:
      if(moveDone(ctl_, /*requireConvergence=*/true))
      {
        mc_rtc::log::info("[Bottle1] Present pose reached, holding. Pull the bottle to release, "
                          "or press \"Return home (safe)\".");
        holdTimer_ = 0.0;
        debounceTimer_ = 0.0;
        earlyDebounceTimer_ = 0.0;
        baselineCaptured_ = false;
        phase_ = HOLDING;
      }
      break;
    case HOLDING:
    {
      holdTimer_ += ctl.timeStep;
      bool release = returnRequested_;
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
            baselineJoint2_ = t2;
            baselineJoint4_ = t4;
            baselineCaptured_ = true;
            lastD2_ = 0.0;
            lastD4_ = 0.0;
            mc_rtc::log::info("[Bottle1] Detection armed, baseline joint_2={:.2f} joint_4={:.2f}", baselineJoint2_,
                              baselineJoint4_);
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
          mc_rtc::log::info("[Bottle1] Pull detected (d_joint_2={:.2f}, d_joint_4={:.2f}, "
                            "abs_joint_2={:.2f}, abs_joint_4={:.2f}, early={}), releasing",
                            lastD2_, lastD4_, t2, t4, viaEarly);
          release = true;
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
        mc_rtc::log::info("[Bottle1] Bottle released, moving straight to home ({:.1f} s)", tf_);
        phase_ = RETURN_TO_HOME;
      }
      break;
    case RETURN_TO_HOME:
      if(moveDone(ctl_, /*requireConvergence=*/true))
      {
        mc_rtc::log::info("[Bottle1] Returned home safely.");
        phase_ = DONE;
      }
      break;
    case DONE:
      output("OK");
      return true;
  }
  return false;
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
}

EXPORT_SINGLE_STATE("KinovaController_Bottle1", KinovaController_Bottle1)
