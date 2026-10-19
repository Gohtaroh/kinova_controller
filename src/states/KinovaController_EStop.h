#pragma once

#include <mc_control/fsm/State.h>

#include <string>
#include <vector>

/** Software emergency stop (2026-10-19).
 *
 *  Entered by interrupting whatever state is running (KinovaController calls
 *  executor_.resume("KinovaController_EStop") when the mouse middle-click
 *  watcher sends ESTOP). On start it drops the EE task and pins the posture
 *  target to the CURRENT commanded joint configuration, so the arm stops where
 *  it is and holds. The gripper is not touched (a held bottle is not dropped).
 *
 *  It stays here until the operator presses the GUI button
 *  "Emergency stop" -> "Return home (safe)": then the arm returns to the home
 *  joint configuration with a slow, folding-corrected minimum-jerk move (same
 *  method as Bottle1/NGPoint, so Initial's one-shot home command afterwards is
 *  harmless) and emits OK (auto transition to Initial).
 */
struct KinovaController_EStop : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  enum Phase
  {
    HOLDING, // stopped, holding the pose at the moment of the stop
    RETURNING, // operator requested the safe return home
    DONE
  };
  Phase phase_ = HOLDING;
  bool returnRequested_ = false;

  double returnSpeed_ = 0.4; // [rad/s] peak joint speed of the safe return (slower than normal)
  double minMoveTime_ = 1.0; // [s]
  double convergenceGrace_ = 3.0; // [s]

  std::vector<double> startQ_;
  std::vector<double> targetQ_;
  double t_ = 0.0;
  double tf_ = 0.0;
};
