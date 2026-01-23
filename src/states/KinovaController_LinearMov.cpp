#include "KinovaController_LinearMov.h"

#include "../KinovaController.h"

void KinovaController_LinearMov::configure(const mc_rtc::Configuration & config) {}

void KinovaController_LinearMov::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  
  // Activate Null Space compliance
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(5.0);
  ctl.compPostureTask->makeCompliant(true);

  // Move forward the end-effector
  ctl.compEETask->reset();
  ctl.compEETask->positionTask->stiffness(50.0);
  ctl.compEETask->positionTask->damping(30.0);
  ctl.compEETask->makeCompliant(false);
  Eigen::Vector3d current_pos = ctl.compEETask->positionTask->position();
  Eigen::Vector3d new_pos = current_pos + Eigen::Vector3d(0.15, 0.0, 0.0);
  ctl.compEETask->positionTask->position(new_pos);
}

bool KinovaController_LinearMov::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);

   if(ctl.compEETask->eval().norm() < 0.05)
   {
     // Move the end-effector in the opposite direction
     moveBackward_ = !moveBackward_;
     if(moveBackward_)
     {
       Eigen::Vector3d current_pos = ctl.compEETask->positionTask->position();
       Eigen::Vector3d new_pos = current_pos + Eigen::Vector3d(-0.15, 0.0, 0.0);
       ctl.compEETask->positionTask->position(new_pos);
     }
     else
     {
      Eigen::Vector3d current_pos = ctl.compEETask->positionTask->position();
      Eigen::Vector3d new_pos = current_pos + Eigen::Vector3d(0.15, 0.0, 0.0);
      ctl.compEETask->positionTask->position(new_pos);
    }
  }
  // output("OK");
  return false;
}

void KinovaController_LinearMov::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  
}

EXPORT_SINGLE_STATE("KinovaController_LinearMov", KinovaController_LinearMov)
