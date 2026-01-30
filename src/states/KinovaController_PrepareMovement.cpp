#include "KinovaController_PrepareMovement.h"

#include "../KinovaController.h"

void KinovaController_PrepareMovement::configure(const mc_rtc::Configuration & config) {}

void KinovaController_PrepareMovement::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  ctl.postureTask->stiffness(100.0);
  ctl.taskPosition = ctl.homePosition;
  ctl.taskOrientation = ctl.taskOrientation;
  ctl.compEETask->positionTask->stiffness(100.0);
  ctl.compEETask->orientationTask->stiffness(100.0);
  ctl.compEETask->positionTask->damping(50.0);
  ctl.compEETask->orientationTask->damping(50.0);
  ctl.compEETask->positionTask->position(ctl.taskPosition);
  ctl.compEETask->orientationTask->orientation(ctl.taskOrientation);
  ctl.solver().addTask(ctl.compEETask);
}

bool KinovaController_PrepareMovement::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  if(ctl.compEETask->eval().norm() < 0.01)
  { 
    output("OK");
    return true;
  }
  return false;
}

void KinovaController_PrepareMovement::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
}

EXPORT_SINGLE_STATE("KinovaController_PrepareMovement", KinovaController_PrepareMovement)
