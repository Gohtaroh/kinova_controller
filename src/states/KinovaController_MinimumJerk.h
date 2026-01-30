#pragma once

#include <mc_control/fsm/State.h>

#define START_POS_STATE 0
#define REPEAT_STATE 1
#define STOP_STATE 2

struct KinovaController_MinimumJerk : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  bool moveBackward_ = false;
  double currentVelocity_ = 0.0;
  int state_ = START_POS_STATE;
  int repeatCount_ = 0;
  int maxRepeats_ = 3;
  double retractVelocity_ = 0.05;
  double approachVelocity_ = 0.4;
};
