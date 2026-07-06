#include "KinovaController_TestSequence.h"

#include "../KinovaController.h"

void KinovaController_TestSequence::configure(const mc_rtc::Configuration & config)
{
  config("rotationAngle", rotationAngle_);
  config("rotationSpeed", rotationSpeed_);
  config("wristJoint", wristJoint_);
}

void KinovaController_TestSequence::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  auto jIndex = ctl.robot().jointIndexByName(wristJoint_);
  startAngle_ = ctl.robot().mbc().q[jIndex][0];
  rampTarget_ = startAngle_;
  goalAngle_ = startAngle_ + rotationAngle_;
  timer_ = 0.0;
  phase_ = ROTATE_WRIST;
  mc_rtc::log::info("[TestSequence] Start: rotating {} by {:.0f} deg", wristJoint_, rotationAngle_ * 180.0 / M_PI);
}

bool KinovaController_TestSequence::rampAndReached(mc_control::fsm::Controller & ctl_, double goal)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  double dir = (goal > rampTarget_) ? 1.0 : -1.0;
  rampTarget_ += dir * rotationSpeed_ * ctl.timeStep;
  if((dir > 0.0 && rampTarget_ >= goal) || (dir < 0.0 && rampTarget_ <= goal))
  {
    rampTarget_ = goal;
  }
  ctl.compPostureTask->target({{wristJoint_, {rampTarget_}}});
  double q = ctl.robot().mbc().q[ctl.robot().jointIndexByName(wristJoint_)][0];
  return rampTarget_ == goal && std::abs(q - goal) < 0.05;
}

bool KinovaController_TestSequence::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  switch(phase_)
  {
    case ROTATE_WRIST:
      if(rampAndReached(ctl_, goalAngle_))
      {
        mc_rtc::log::info("[TestSequence] Wrist rotation done, closing gripper");
        ctl.robot().gripper("gripper").setTargetOpening(0.0);
        timer_ = 0.0;
        phase_ = CLOSE_GRIPPER;
      }
      break;
    case CLOSE_GRIPPER:
      timer_ += ctl.timeStep;
      if(timer_ > gripperSettle_ && ctl.robot().gripper("gripper").complete())
      {
        mc_rtc::log::info("[TestSequence] Gripper closed, opening gripper");
        ctl.robot().gripper("gripper").setTargetOpening(1.0);
        timer_ = 0.0;
        phase_ = OPEN_GRIPPER;
      }
      break;
    case OPEN_GRIPPER:
      timer_ += ctl.timeStep;
      if(timer_ > gripperSettle_ && ctl.robot().gripper("gripper").complete())
      {
        mc_rtc::log::info("[TestSequence] Gripper opened, rotating wrist back");
        phase_ = ROTATE_BACK;
      }
      break;
    case ROTATE_BACK:
      if(rampAndReached(ctl_, startAngle_))
      {
        mc_rtc::log::info("[TestSequence] Sequence complete");
        phase_ = DONE;
      }
      break;
    case DONE:
      output("OK");
      return true;
  }
  return false;
}

void KinovaController_TestSequence::teardown(mc_control::fsm::Controller & ctl_) {}

EXPORT_SINGLE_STATE("KinovaController_TestSequence", KinovaController_TestSequence)
