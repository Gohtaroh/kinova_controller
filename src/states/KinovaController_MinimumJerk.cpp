#include "KinovaController_MinimumJerk.h"

#include "../KinovaController.h"

void KinovaController_MinimumJerk::configure(const mc_rtc::Configuration & config) {}

void KinovaController_MinimumJerk::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  moveBackward_ = true;
  currentVelocity_ = retractVelocity_;
  state_ = START_POS_STATE;
  repeatCount_ = 0;
  ctl.initializeMinimumJerkMove(ctl.cardPosition_1, 0.5, currentVelocity_);

}

bool KinovaController_MinimumJerk::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);

  if(!ctl.motionComplete()) ctl.minimumJerkMove();
  else
  {
    switch (state_) {
      case START_POS_STATE:
        moveBackward_ = false;
        currentVelocity_ = approachVelocity_;
        mc_rtc::log::info("[KinovaController_MinimumJerk] Moving forward to card.");
        ctl.initializeMinimumJerkMove(ctl.cardPosition_1, 0.05, currentVelocity_);
        state_ = REPEAT_STATE;
        break;
      case REPEAT_STATE:
        if(repeatCount_ < maxRepeats_)
        {
          moveBackward_ = !moveBackward_;
          if(moveBackward_)
          {
            currentVelocity_ = retractVelocity_;
            ctl.initializeMinimumJerkMove(ctl.cardPosition_1, 0.15, currentVelocity_);
            mc_rtc::log::info("[KinovaController_MinimumJerk] Moving backward to card.");
          }
          else
          {
            currentVelocity_ = approachVelocity_;
            ctl.initializeMinimumJerkMove(ctl.cardPosition_1, 0.05, currentVelocity_);
            mc_rtc::log::info("[KinovaController_MinimumJerk] Moving forward to card.");
            repeatCount_++;
            mc_rtc::log::info("[KinovaController_MinimumJerk] Repeat count: {}", repeatCount_);
          }
        }
        else
        {
          state_ = STOP_STATE;
        }
        break;
      case STOP_STATE:
        // mc_rtc::log::info("[KinovaController_MinimumJerk] MinimumJerk movement complete.");
        output("OK");
        return true;
      default:
        break;
    
    }
  }
  return false;
}

void KinovaController_MinimumJerk::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  ctl.directionVec = ctl.basedDirectionVec;
}

EXPORT_SINGLE_STATE("KinovaController_MinimumJerk", KinovaController_MinimumJerk)
