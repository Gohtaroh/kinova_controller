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
    MOVE_TO_APPROACH,
    MOVE_TO_GRASP,
    CLOSE_GRIPPER,
    MOVE_TO_LIFTUP,
    MOVE_TO_PRESENT,
    HOLDING,
    RELEASING,
    RETURN_TO_HOME,
    DONE
  };
  Phase phase_ = MOVE_TO_APPROACH;

  // Parameters (overridable from the FSM state configuration)
  // Approach waypoint above the bottle: descending from here to poseA keeps the
  // end-effector path top-down instead of bulging forward into the bottle top.
  std::vector<double> poseApproachDeg_ = {359.707, 59.4, 180.031, 214.838, 359.63, 114.512, 88.379};
  std::vector<double> poseADeg_ = {359.501, 81.853, 180.238, 238.467, 359.126, 113.316, 88.062};
  // Lift-up waypoint just above the grasp pose: raise the bottle straight up
  // before swinging it forward, so it does not drag along the ground.
  std::vector<double> poseLiftDeg_ = {359.72, 52.925, 180.193, 224.675, 359.867, 98.206, 88.765};
  // Present/handover pose: lift the bottle and reorient the wrist toward the participant
  std::vector<double> poseBDeg_ = {1.451, 88.829, 188.223, 358.957, 356.36, 0.63, 85.62};
  double gripperClosedPct_ = 43.0; // [%] Kinova convention: 0 = fully open, 100 = fully closed
  double jointSpeed_ = 0.4; // [rad/s] peak joint velocity for the grasp/lift moves
  // The approach (home -> waypoint) sweeps a large joint angle; give it a higher
  // peak velocity so it does not drag compared to the short grasp descent.
  double approachSpeed_ = 0.8; // [rad/s] peak joint velocity for the approach move
  // The lift/present move also sweeps large joint angles; keep it brisk so the
  // handover pose is reached quickly (the grasp descent stays at jointSpeed_).
  double liftSpeed_ = 0.8; // [rad/s] peak joint velocity for the lift/present move
  // Return-to-home goes straight from the present pose to home (wrapped so every
  // joint takes the short way); keep it gentle since it happens near the participant.
  double returnSpeed_ = 0.8; // [rad/s] peak joint velocity for the return-home move

  // Pull-to-release detection at the present pose. The extended arm + bottle
  // weight puts a large steady bias on the external-torque estimate (~1.04 on
  // joint_4, ~-0.63 on joint_2), so we can't threshold the raw value. Instead we
  // capture that resting value as a baseline once the arm settles, and trigger on
  // the CHANGE from the baseline when the participant pulls. Resting change ~0,
  // pull change ~1.8 (joint_4) / ~3.8 (joint_2), so 0.5 is sensitive and safe.
  double pullJoint4Thresh_ = 0.5; // joint_4 rise above baseline = pull (front/back)
  double pullJoint2Thresh_ = 0.5; // |joint_2 change| from baseline = pull (up/down)
  // Absolute (baseline-independent) fallback: catches a strong pull even if the
  // baseline was polluted by an impatient participant already pulling at capture
  // time. Set well above the resting bias (joint_4 ~1.04, |joint_2| ~0.63) but
  // below the pull peaks (joint_4 ~2.87, |joint_2| ~4.43).
  double pullJoint4Abs_ = 2.0; // joint_4 raw estimate above this = pull, regardless of baseline
  double pullJoint2Abs_ = 2.5; // |joint_2| raw estimate above this = pull, regardless of baseline
  // Early strong-pull thresholds, checked even during the settle window (before the
  // baseline arms) so a participant who pulls immediately still gets the bottle. The
  // abrupt Tool stop transient reaches the same magnitude as a pull (|t2| ~4), so an
  // absolute level alone can't separate them; instead we lean on DURATION: the
  // transient is choppy (dithers, longest run ~0.13 s) while a real pull is sustained,
  // so the early path requires a longer continuous hold (earlyDebounceTime_) than the
  // post-arm path. The level is set just below the real pull (~3.3) to catch it.
  double earlyPullJoint4Abs_ = 2.3; // joint_4 raw estimate above this = candidate pull, during the settle window
  double earlyPullJoint2Abs_ = 2.8; // |joint_2| raw estimate above this = candidate pull, during the settle window
  double earlyDebounceTime_ =
      0.35; // [s] early threshold must hold continuously this long (rejects the choppy transient)
  double armDelay_ = 0.2; // [s] settle time after reaching the present pose before arming detection
  double debounceTime_ = 0.15; // [s] threshold must hold continuously this long to fire
  double gripperSettle_ = 0.5; // [s] minimum wait before checking gripper completion
  double convergenceGrace_ = 3.0; // [s] extra time allowed to converge before moving on

  // Trajectory style. Both are ALWAYS overwritten in start() from the controller's
  // active handover condition, so these initializers are only a fallback for
  // standalone use. alpha_ blends the joint-space time scaling: 0 = constant
  // velocity (mechanical), 1 = minimum jerk (human-like, = the verified 2026-07-06
  // profile). speedScale_ multiplies every move speed so the whole sequence runs at
  // one tempo. The active default is whatever condition the controller selects
  // (see KinovaController handoverCondition_), NOT necessarily these values.
  double alpha_ = 1.0;
  double speedScale_ = 1.0;

  std::vector<double> startQ_;
  std::vector<double> targetQ_;
  double t_ = 0.0;
  double tf_ = 0.0;
  double timer_ = 0.0;
  bool returnRequested_ = false; // set by the GUI "Return home" button
  double holdTimer_ = 0.0; // [s] time spent holding at the present pose
  double debounceTimer_ = 0.0; // [s] time the (post-arm) pull threshold has been continuously exceeded
  double earlyDebounceTimer_ = 0.0; // [s] time the early strong-pull threshold has been continuously exceeded
  int dofJoint2_ = -1; // DoF-vector index of joint_2 in the external-torque estimate
  int dofJoint4_ = -1; // DoF-vector index of joint_4 in the external-torque estimate
  bool forceDetectAvailable_ = false; // whether the estimator read-call is present
  bool baselineCaptured_ = false; // whether the resting external-torque baseline is set
  double baselineJoint2_ = 0.0; // resting joint_2 external torque at the present pose
  double baselineJoint4_ = 0.0; // resting joint_4 external torque at the present pose
  // Latest pull-detection signals, exposed to the mc_rtc logger during HOLDING so the
  // settling drift and pull can be plotted and the thresholds/settle times tuned.
  double lastTau2_ = 0.0; // latest joint_2 external-torque estimate
  double lastTau4_ = 0.0; // latest joint_4 external-torque estimate
  double lastD2_ = 0.0; // latest joint_2 deviation from baseline
  double lastD4_ = 0.0; // latest joint_4 deviation from baseline

  /** Start a minimum-jerk posture move to poseDeg. If continuous, the move starts
   *  from the previous segment's commanded end (for a hitch-free waypoint chain);
   *  otherwise it starts from the current actual joint values. */
  void initMove(mc_control::fsm::Controller & ctl,
                const std::vector<double> & poseDeg,
                double speed,
                bool continuous = false);
  /** Advance the interpolated posture target. With requireConvergence, returns true
   *  only once the joints settle near the target (for final poses); without it,
   *  returns true as soon as the minimum-jerk time elapses (for via-points). */
  bool moveDone(mc_control::fsm::Controller & ctl, bool requireConvergence = true);
};
