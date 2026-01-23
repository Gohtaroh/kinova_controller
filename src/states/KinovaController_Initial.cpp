#include "KinovaController_Initial.h"

#include "../KinovaController.h"

void KinovaController_Initial::configure(const mc_rtc::Configuration & config) {}

void KinovaController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  // Enable feedback from external forces estimator
  if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Setting residual gain of external forces estimator
  ctl.datastore().call<void, double>("EF_Estimator::setGain", 30.0);
}

bool KinovaController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  // output("LinearMov");
  // return true; to transition
  return false;
}

void KinovaController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
}

EXPORT_SINGLE_STATE("KinovaController_Initial", KinovaController_Initial)
