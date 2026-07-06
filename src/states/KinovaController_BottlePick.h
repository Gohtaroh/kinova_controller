#pragma once

#include <mc_control/fsm/State.h>

#include <string>
#include <vector>

/** One-button bottle pick prototype:
 *  1. open the gripper and move to poseA (grasp pose)
 *  2. close the gripper to the configured closure percentage
 *  3. move to poseB (lift pose) and hold there until the user triggers the
 *     transition back to Initial
 *
 *  Poses are 7 joint angles in the Kinova web-console convention (degrees,
 *  [0, 360)); they are converted to the wrap-free representation closest to
 *  the current joint value, so continuous joints never take the long way
 *  around. Moves use minimum-jerk time scaling in joint space.
 *
 *  Joint-space only: does not rely on compEETask, safe to force-transition
 *  into. poseA_deg / poseB_deg / gripperClosedPct / jointSpeed can be
 *  overridden from the FSM configuration to create more pick locations
 *  without new code.
 */
struct KinovaController_BottlePick : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  enum Phase
  {
    MOVE_TO_GRASP,
    CLOSE_GRIPPER,
    MOVE_TO_LIFT,
    DONE
  };
  Phase phase_ = MOVE_TO_GRASP;

  // Parameters (overridable from the FSM state configuration)
  std::vector<double> poseADeg_ = {359.501, 81.853, 180.238, 238.467, 359.126, 113.316, 88.062};
  std::vector<double> poseBDeg_ = {359.814, 41.841, 180.125, 223.564, 0.033, 88.244, 89.107};
  double gripperClosedPct_ = 43.0; // [%] Kinova convention: 0 = fully open, 100 = fully closed
  double jointSpeed_ = 0.4; // [rad/s] peak joint velocity of the interpolated target
  double gripperSettle_ = 0.5; // [s] minimum wait before checking gripper completion
  double convergenceGrace_ = 3.0; // [s] extra time allowed to converge before moving on

  std::vector<double> startQ_;
  std::vector<double> targetQ_;
  double t_ = 0.0;
  double tf_ = 0.0;
  double timer_ = 0.0;

  void initMove(mc_control::fsm::Controller & ctl, const std::vector<double> & poseDeg);
  /** Advance the interpolated posture target, returns true once the joints reached it */
  bool moveDone(mc_control::fsm::Controller & ctl);
};
