#include "KinovaController_BottlePick.h"

#include "../KinovaController.h"

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Button.h>

#include <Eigen/Core>

static const std::vector<std::string> jointNames = {"joint_1", "joint_2", "joint_3", "joint_4",
                                                    "joint_5", "joint_6", "joint_7"};

void KinovaController_BottlePick::configure(const mc_rtc::Configuration & config)
{
  config("poseApproach_deg", poseApproachDeg_);
  config("poseA_deg", poseADeg_);
  config("poseLift_deg", poseLiftDeg_);
  config("poseB_deg", poseBDeg_);
  config("gripperClosedPct", gripperClosedPct_);
  config("jointSpeed", jointSpeed_);
  config("approachSpeed", approachSpeed_);
  config("liftSpeed", liftSpeed_);
  config("returnSpeed", returnSpeed_);
  config("pullJoint4Thresh", pullJoint4Thresh_);
  config("pullJoint2Thresh", pullJoint2Thresh_);
  config("pullJoint4Abs", pullJoint4Abs_);
  config("pullJoint2Abs", pullJoint2Abs_);
  config("armDelay", armDelay_);
  config("debounceTime", debounceTime_);
}

void KinovaController_BottlePick::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  // Adopt the active handover condition (motion style) for the whole sequence, so
  // the pick, lift, present and return all share one tempo and smoothness.
  alpha_ = ctl.handoverAlpha();
  speedScale_ = ctl.handoverSpeedScale();
  // Sharper stops (alpha < 1) need a longer settle before the pull-detection baseline
  // is captured, else the arrival transient is misread as a pull (bottle self-releases).
  armDelay_ = ctl.handoverArmDelay();
  mc_rtc::log::info("[BottlePick] Handover condition: {} (speed x{:.3f}, alpha {:.2f}, armDelay {:.2f}s)",
                    ctl.handoverConditionName(), speedScale_, alpha_, armDelay_);
  // Self-contained: this state only uses the posture task
  ctl.solver().removeTask(ctl.compEETask);
  ctl.robot().gripper("gripper").setTargetOpening(1.0);
  returnRequested_ = false;
  // Operator-triggered safe return: retraces the taught waypoints back to home
  // instead of letting Initial snap there directly (which takes joints the long
  // way around and trips the e-stop).
  ctl.gui()->addElement({"BottlePick"},
                        mc_rtc::gui::Button("Return home (safe)", [this]() { returnRequested_ = true; }));

  // Log the pull-detection signals so the settling drift and the pull are plottable
  // (mc_rtc log / plot), for tuning the per-condition settle time and thresholds.
  ctl.logger().addLogEntry("BottlePick_tau2", [this]() { return lastTau2_; });
  ctl.logger().addLogEntry("BottlePick_tau4", [this]() { return lastTau4_; });
  ctl.logger().addLogEntry("BottlePick_d2", [this]() { return lastD2_; });
  ctl.logger().addLogEntry("BottlePick_d4", [this]() { return lastD4_; });

  // Resolve where joint_2 / joint_4 live in the DoF vector so we can index the
  // external-torque estimate correctly, and check the estimator read-call exists.
  dofJoint2_ = ctl.robot().mb().jointPosInDof(ctl.robot().jointIndexByName("joint_2"));
  dofJoint4_ = ctl.robot().mb().jointPosInDof(ctl.robot().jointIndexByName("joint_4"));
  forceDetectAvailable_ = ctl.datastore().has("EF_Estimator::getExternalTorques");
  if(!forceDetectAvailable_)
  {
    mc_rtc::log::warning("[BottlePick] ExternalForcesEstimator read-call not found; "
                         "pull-to-release disabled, use the Return home button");
  }

  initMove(ctl_, poseApproachDeg_, approachSpeed_);
  phase_ = MOVE_TO_APPROACH;
  mc_rtc::log::info("[BottlePick] Moving to approach pose ({:.1f} s)", tf_);
}

void KinovaController_BottlePick::initMove(mc_control::fsm::Controller & ctl_,
                                           const std::vector<double> & poseDeg,
                                           double speed,
                                           bool continuous)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  constexpr double TWO_PI = 2.0 * mc_rtc::constants::PI;
  // For a continuous chain of moves, start this segment from the previous
  // segment's commanded end (targetQ_) rather than the lagging actual joint
  // value, so the posture command stays continuous through the waypoint (no
  // backward jump / hitch). The first move of the sequence starts from actual q.
  std::vector<double> prevTarget = targetQ_;
  startQ_.clear();
  targetQ_.clear();
  double maxDelta = 0.0;
  for(size_t i = 0; i < jointNames.size(); ++i)
  {
    double ref = (continuous && i < prevTarget.size())
                     ? prevTarget[i]
                     : ctl.robot().mbc().q[ctl.robot().jointIndexByName(jointNames[i])][0];
    // Kinova web-console angles live in [0, 360); pick the equivalent angle
    // closest to the current reference so no joint takes the long way around
    double target = mc_rtc::constants::toRad(poseDeg[i]);
    target += TWO_PI * std::round((ref - target) / TWO_PI);
    startQ_.push_back(ref);
    targetQ_.push_back(target);
    maxDelta = std::max(maxDelta, std::abs(target - ref));
  }
  // speed is the target peak joint velocity. The blended profile's peak/mean
  // velocity ratio is (1 + 0.875*alpha) (1.0 for constant velocity, 1.875 for
  // minimum jerk), so tf = ratio * distance / peak_speed keeps `speed` meaning
  // peak velocity across all alpha. speedScale_ applies the condition's tempo.
  const double peakFactor = 1.0 + 0.875 * alpha_;
  tf_ = std::max(peakFactor * maxDelta / (speed * speedScale_), 0.5);
  t_ = 0.0;
}

bool KinovaController_BottlePick::moveDone(mc_control::fsm::Controller & ctl_, bool requireConvergence)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  t_ += ctl.timeStep;
  double u = std::min(t_ / tf_, 1.0);
  // Blend constant-velocity (u) and minimum-jerk (quintic) time scalings by alpha_.
  // alpha 0 -> constant velocity (sharp start/stop), alpha 1 -> minimum jerk (eased).
  // s(0)=0, s(1)=1 and ds/du >= 0 for all alpha in [0,1], so the target is
  // continuous and never reverses. The endpoint rate scales with (1-alpha), which
  // is the perceptual "kick" cue the conditions vary.
  double mj = u * u * u * (10.0 + u * (-15.0 + 6.0 * u)); // minimum-jerk time scaling
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
  // Waypoints (via-points) do not need to settle: proceed as soon as the
  // minimum-jerk time is up, so the motion flows into the next segment instead
  // of stalling in the convergence wait. Final poses still wait to converge.
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
    mc_rtc::log::warning("[BottlePick] Move did not fully converge (max joint error {:.3f} rad), continuing", maxErr);
    return true;
  }
  return false;
}

bool KinovaController_BottlePick::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  switch(phase_)
  {
    case MOVE_TO_APPROACH:
      // Via-point: do not wait to converge, flow straight into the descent
      if(moveDone(ctl_, /*requireConvergence=*/false))
      {
        mc_rtc::log::info("[BottlePick] Approach pose reached, descending to grasp pose");
        initMove(ctl_, poseADeg_, jointSpeed_, /*continuous=*/true);
        phase_ = MOVE_TO_GRASP;
      }
      break;
    case MOVE_TO_GRASP:
      if(moveDone(ctl_))
      {
        mc_rtc::log::info("[BottlePick] Grasp pose reached, closing gripper to {:.0f}%", gripperClosedPct_);
        ctl.robot().gripper("gripper").setTargetOpening(1.0 - gripperClosedPct_ / 100.0);
        timer_ = 0.0;
        phase_ = CLOSE_GRIPPER;
      }
      break;
    case CLOSE_GRIPPER:
      timer_ += ctl.timeStep;
      if((timer_ > gripperSettle_ && ctl.robot().gripper("gripper").complete()) || timer_ > 4.0)
      {
        if(timer_ > 4.0)
        {
          mc_rtc::log::warning("[BottlePick] Gripper close did not report completion, continuing anyway");
        }
        initMove(ctl_, poseLiftDeg_, liftSpeed_, /*continuous=*/true);
        mc_rtc::log::info("[BottlePick] Gripper closed, lifting bottle straight up ({:.1f} s)", tf_);
        phase_ = MOVE_TO_LIFTUP;
      }
      break;
    case MOVE_TO_LIFTUP:
      // Via-point: lift the bottle clear of the ground, then flow into the present move
      if(moveDone(ctl_, /*requireConvergence=*/false))
      {
        mc_rtc::log::info("[BottlePick] Bottle lifted, moving to present pose");
        initMove(ctl_, poseBDeg_, liftSpeed_, /*continuous=*/true);
        phase_ = MOVE_TO_PRESENT;
      }
      break;
    case MOVE_TO_PRESENT:
      if(moveDone(ctl_))
      {
        mc_rtc::log::info("[BottlePick] Present pose reached, holding. Pull the bottle to release, "
                          "or press \"Return home (safe)\".");
        holdTimer_ = 0.0;
        debounceTimer_ = 0.0;
        baselineCaptured_ = false;
        phase_ = HOLDING;
      }
      break;
    case HOLDING:
    {
      holdTimer_ += ctl.timeStep;
      // The operator button is always a valid manual trigger.
      bool release = returnRequested_;
      // Pull detection: only after a short settle (the move-stop transient pollutes
      // the estimate), and only once the read-call is available.
      if(!release && forceDetectAvailable_ && holdTimer_ > armDelay_)
      {
        const Eigen::VectorXd tauExt = ctl.datastore().call<Eigen::VectorXd>("EF_Estimator::getExternalTorques");
        double t2 = tauExt[dofJoint2_];
        double t4 = tauExt[dofJoint4_];
        lastTau2_ = t2;
        lastTau4_ = t4;
        if(!baselineCaptured_)
        {
          // Capture the resting bias (extended arm + bottle weight) as the zero
          // reference, so detection triggers on the change caused by a pull.
          baselineJoint2_ = t2;
          baselineJoint4_ = t4;
          baselineCaptured_ = true;
          mc_rtc::log::info("[BottlePick] Detection armed, baseline joint_2={:.2f} joint_4={:.2f}", baselineJoint2_,
                            baselineJoint4_);
        }
        else
        {
          double d2 = t2 - baselineJoint2_;
          double d4 = t4 - baselineJoint4_;
          lastD2_ = d2;
          lastD4_ = d4;
          // Primary (weight-robust): change from the resting baseline captured for
          // THIS bottle, so a heavier/lighter condiment doesn't matter. The baseline
          // is captured only after armDelay_ (long for the abrupt Tool condition) so
          // the post-stop settling transient has died before it is latched.
          bool deltaOver = (d4 > pullJoint4Thresh_) || (d2 > pullJoint2Thresh_) || (d2 < -pullJoint2Thresh_);
          // Fallback (baseline-independent): strong pull that a polluted baseline
          // would otherwise hide (participant already pulling at capture time).
          bool absOver = (t4 > pullJoint4Abs_) || (t2 > pullJoint2Abs_) || (t2 < -pullJoint2Abs_);
          bool over = deltaOver || absOver;
          debounceTimer_ = over ? debounceTimer_ + ctl.timeStep : 0.0;
          if(debounceTimer_ > debounceTime_)
          {
            mc_rtc::log::info("[BottlePick] Pull detected (d_joint_2={:.2f}, d_joint_4={:.2f}, "
                              "abs_joint_2={:.2f}, abs_joint_4={:.2f}), releasing",
                              d2, d4, t2, t4);
            release = true;
          }
        }
      }
      if(release)
      {
        // Release the bottle, then return home once the gripper has opened.
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
        // Go straight home. Build the home posture (stored in radians) as degrees
        // so the wrapped minimum-jerk move takes every joint the short way back —
        // this is what keeps a direct return from taking joints the long way
        // around and tripping the e-stop, without detouring via the pickup pose.
        std::vector<double> homeDeg;
        for(const auto & jn : jointNames)
        {
          homeDeg.push_back(ctl.postureHome.at(jn)[0] * 180.0 / mc_rtc::constants::PI);
        }
        initMove(ctl_, homeDeg, returnSpeed_, /*continuous=*/true);
        mc_rtc::log::info("[BottlePick] Bottle released, moving straight to home ({:.1f} s)", tf_);
        phase_ = RETURN_TO_HOME;
      }
      break;
    case RETURN_TO_HOME:
      if(moveDone(ctl_))
      {
        mc_rtc::log::info("[BottlePick] Returned home safely.");
        phase_ = DONE;
      }
      break;
    case DONE:
      output("OK");
      return true;
  }
  return false;
}

void KinovaController_BottlePick::teardown(mc_control::fsm::Controller & ctl_)
{
  ctl_.gui()->removeElement({"BottlePick"}, "Return home (safe)");
  ctl_.logger().removeLogEntry("BottlePick_tau2");
  ctl_.logger().removeLogEntry("BottlePick_tau4");
  ctl_.logger().removeLogEntry("BottlePick_d2");
  ctl_.logger().removeLogEntry("BottlePick_d4");
}

EXPORT_SINGLE_STATE("KinovaController_BottlePick", KinovaController_BottlePick)
