#pragma once

#include <mc_control/fsm/State.h>

#include <cmath>

/** One-button demo sequence (runs to completion without further input):
 *  1. rotate the wrist joint by rotationAngle (target is ramped at rotationSpeed)
 *  2. close the gripper
 *  3. open the gripper
 *  4. rotate the wrist back, then return to Initial automatically
 *
 *  Joint-space only: does not rely on compEETask, so it is safe to trigger
 *  directly from KinovaController_Initial.
 */
struct KinovaController_TestSequence : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  enum Phase
  {
    ROTATE_WRIST,
    CLOSE_GRIPPER,
    OPEN_GRIPPER,
    ROTATE_BACK,
    DONE
  };
  Phase phase_ = ROTATE_WRIST;

  // Parameters (overridable from the FSM state configuration)
  double rotationAngle_ = M_PI; // [rad] wrist rotation, default 180 deg
  double rotationSpeed_ = 0.8; // [rad/s] speed of the ramped joint target
  double gripperSettle_ = 0.5; // [s] minimum wait before checking gripper completion
  std::string wristJoint_ = "joint_7";

  double startAngle_ = 0.0;
  double rampTarget_ = 0.0;
  double goalAngle_ = 0.0;
  double timer_ = 0.0;

  /** Advance the ramped target towards goal, returns true once the joint reached it */
  bool rampAndReached(mc_control::fsm::Controller & ctl, double goal);
};
