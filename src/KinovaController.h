#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <mc_tasks/CompliantEndEffectorTask.h>

#include "api.h"

struct KinovaController_DLLAPI KinovaController : public mc_control::fsm::Controller
{
  KinovaController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> compEETask;

private:
  mc_rtc::Configuration config_;
};
