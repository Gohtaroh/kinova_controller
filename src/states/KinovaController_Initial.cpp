#include "KinovaController_Initial.h"

#include "../KinovaController.h"

void KinovaController_Initial::configure(const mc_rtc::Configuration & config) {}

void KinovaController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
}

bool KinovaController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  // output("OK");
  return false;
}

void KinovaController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
}

EXPORT_SINGLE_STATE("KinovaController_Initial", KinovaController_Initial)
