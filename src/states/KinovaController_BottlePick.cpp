#include "KinovaController_BottlePick.h"

#include "../KinovaController.h"

#include <mc_rtc/constants.h>

static const std::vector<std::string> jointNames = {"joint_1", "joint_2", "joint_3", "joint_4",
                                                    "joint_5", "joint_6", "joint_7"};

void KinovaController_BottlePick::configure(const mc_rtc::Configuration & config)
{
  config("poseA_deg", poseADeg_);
  config("poseB_deg", poseBDeg_);
  config("gripperClosedPct", gripperClosedPct_);
  config("jointSpeed", jointSpeed_);
}

void KinovaController_BottlePick::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  // Self-contained: this state only uses the posture task
  ctl.solver().removeTask(ctl.compEETask);
  ctl.robot().gripper("gripper").setTargetOpening(1.0);
  initMove(ctl_, poseADeg_);
  phase_ = MOVE_TO_GRASP;
  mc_rtc::log::info("[BottlePick] Moving to grasp pose ({:.1f} s)", tf_);
}

void KinovaController_BottlePick::initMove(mc_control::fsm::Controller & ctl_, const std::vector<double> & poseDeg)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  constexpr double TWO_PI = 2.0 * mc_rtc::constants::PI;
  startQ_.clear();
  targetQ_.clear();
  double maxDelta = 0.0;
  for(size_t i = 0; i < jointNames.size(); ++i)
  {
    double q = ctl.robot().mbc().q[ctl.robot().jointIndexByName(jointNames[i])][0];
    // Kinova web-console angles live in [0, 360); pick the equivalent angle
    // closest to the current joint value so no joint takes the long way around
    double target = mc_rtc::constants::toRad(poseDeg[i]);
    target += TWO_PI * std::round((q - target) / TWO_PI);
    startQ_.push_back(q);
    targetQ_.push_back(target);
    maxDelta = std::max(maxDelta, std::abs(target - q));
  }
  // 1.875 = peak velocity factor of the minimum jerk profile
  tf_ = std::max(1.875 * maxDelta / jointSpeed_, 0.5);
  t_ = 0.0;
}

bool KinovaController_BottlePick::moveDone(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  t_ += ctl.timeStep;
  double u = std::min(t_ / tf_, 1.0);
  double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u)); // minimum jerk time scaling
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
        initMove(ctl_, poseBDeg_);
        mc_rtc::log::info("[BottlePick] Gripper closed, moving to lift pose ({:.1f} s)", tf_);
        phase_ = MOVE_TO_LIFT;
      }
      break;
    case MOVE_TO_LIFT:
      if(moveDone(ctl_))
      {
        mc_rtc::log::info("[BottlePick] Lift pose reached, holding. Trigger OK to return to Initial.");
        phase_ = DONE;
      }
      break;
    case DONE:
      output("OK");
      return true;
  }
  return false;
}

void KinovaController_BottlePick::teardown(mc_control::fsm::Controller & ctl_) {}

EXPORT_SINGLE_STATE("KinovaController_BottlePick", KinovaController_BottlePick)
