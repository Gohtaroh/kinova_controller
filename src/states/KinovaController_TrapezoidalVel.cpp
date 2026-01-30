#include "KinovaController_TrapezoidalVel.h"

#include "../KinovaController.h"

void KinovaController_TrapezoidalVel::configure(const mc_rtc::Configuration & config) {}

void KinovaController_TrapezoidalVel::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  moveBackward_ = true;
  currentVelocity_ = retractVelocity_;
  state_ = START_POS_STATE;
  repeatCount_ = 0;
  ctl.initializeTrapezoidalVelocityMove(ctl.cardPosition_1, 0.5, currentVelocity_, aMax_);
}

bool KinovaController_TrapezoidalVel::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);

  if(!ctl.motionComplete()) ctl.trapezoidalVelocityMove();
  else
  {
    switch (state_) {
      case START_POS_STATE:
        moveBackward_ = false;
        currentVelocity_ = approachVelocity_;
        mc_rtc::log::info("[KinovaController_TrapezoidalVel] Moving forward to card.");
        ctl.initializeTrapezoidalVelocityMove(ctl.cardPosition_1, 0.05, currentVelocity_, aMax_);
        state_ = REPEAT_STATE;
        break;
      case REPEAT_STATE:
        if(repeatCount_ < maxRepeats_)
        {
          moveBackward_ = !moveBackward_;
          if(moveBackward_)
          {
            currentVelocity_ = retractVelocity_;
            ctl.initializeTrapezoidalVelocityMove(ctl.cardPosition_1, 0.15, currentVelocity_, aMax_);
            mc_rtc::log::info("[KinovaController_TrapezoidalVel] Moving backward to card.");
          }
          else
          {
            currentVelocity_ = approachVelocity_;
            ctl.initializeTrapezoidalVelocityMove(ctl.cardPosition_1, 0.05, currentVelocity_, aMax_);
            mc_rtc::log::info("[KinovaController_TrapezoidalVel] Moving forward to card.");
            repeatCount_++;
            mc_rtc::log::info("[KinovaController_TrapezoidalVel] Repeat count: {}", repeatCount_);
          }
        }
        else
        {
          state_ = STOP_STATE;
        }
        break;
      case STOP_STATE:
        // mc_rtc::log::info("[KinovaController_TrapezoidalVel] Trapezoidal velocity movement complete.");
        output("OK");
        return true;
      default:
        break;
    
    }
  }
  return false;
}

void KinovaController_TrapezoidalVel::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  ctl.directionVec = ctl.basedDirectionVec;
}

EXPORT_SINGLE_STATE("KinovaController_TrapezoidalVel", KinovaController_TrapezoidalVel)
