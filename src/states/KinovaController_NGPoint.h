#pragma once

#include <mc_control/fsm/State.h>

#include <Eigen/Core>

#include <vector>

/** One-button NG pointing gesture (experimenter-triggered "that one is wrong").
 *
 *  Positions and flow follow Bastien's card-pointing movement modes
 *  (ConstantVel/TrapezoidalVel/MinimumJerk_FSM): the end-effector moves to a
 *  waypoint startDistance in front of the card (offset along the controller's
 *  basedDirectionVec, like initializeMinimumJerkMove), jabs to pointDistance
 *  from the card, pulls back to retractDistance and jabs again (first jab +
 *  maxRepeats more, matching the original 1 + 3), then returns home.
 *
 *  The motion style however matches BottlePick's handover conditions: every
 *  straight-line segment uses the blended time scaling
 *  s_alpha(u) = (1-alpha)*u + alpha*(10u^3-15u^4+6u^5), with the peak speed
 *  multiplied by the condition's speed scale. Both are read at start() from
 *  the controller's Handover -> Condition GUI selection (Tool/Baseline/
 *  Partner), so the NG gesture and the handover share one motion style per
 *  condition.
 *
 *  Self-contained: manages the compliant EE task itself (Initial and
 *  BottlePick leave it out of the solver), starts with a profiled move to the
 *  EE home position, and ends with a profiled return home, so it is safe to
 *  force-transition into and never leaves Initial a long snap-back to do.
 */
struct KinovaController_NGPoint : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  enum Phase
  {
    CLOSE_GRIPPER, // close the gripper into a pointing fist before any motion
    MOVE_TO_HOME, // profiled entry move to the EE home position (safe from any pose)
    MOVE_TO_START, // home -> waypoint startDistance_ in front of the card
    POINT, // jab towards the card (to pointDistance_)
    RETRACT, // pull back to retractDistance_ between jabs
    RETURN_TO_HOME, // profiled move back to the EE home position
    OPEN_GRIPPER, // reopen the gripper once the gesture is done
    DONE
  };
  Phase phase_ = MOVE_TO_HOME;

  // Parameters (overridable from the FSM state configuration).
  // Geometry and gesture speeds keep Bastien's card-pointing values: start
  // 0.5 m out, jab to 0.05 m at 0.4 m/s, pull back to 0.15 m at 0.05 m/s
  // (quick jab, slow pull-back), 1 + 3 jabs. Only the entry/return positioning
  // moves get their own defaults (the original entered via PrepareMovement and
  // let Initial snap the posture home instead).
  int cardIndex_ = 1; // which controller card position to point at (1-3); 1 = cardPosition_1 (robot-side + to the side)
  std::vector<double> cardPosition_ = {}; // optional explicit card override [m], size 3
  double startDistance_ = 0.5; // [m] first waypoint distance from the card
  double pointDistance_ = 0.05; // [m] closest distance to the card
  double retractDistance_ = 0.15; // [m] pull-back distance between jabs
  int maxRepeats_ = 3; // extra jabs after the first one (total jabs = 1 + maxRepeats)
  // Peak Cartesian speeds [m/s], before the condition speed scale is applied.
  // The positioning moves (prep/start/retract/return) were raised from
  // Bastien's originals (0.05-0.1) after MuJoCo trials: at those values the
  // Partner condition (x0.8, alpha 1) dragged the gesture out too long for an
  // NG cue. They stay well below the verified 0.4 m/s jab band.
  double prepSpeed_ = 0.10; // entry move to the EE home position
  double startSpeed_ = 0.10; // home -> start waypoint
  // Jab speed, lowered twice: 0.4 m/s verified band -> 0.2 (MuJoCo, Tool looked
  // aggressive) -> 0.14 (2026-07-23 real robot: Tool's abrupt jabs shook the
  // bottle bench, so the whole gesture was slowed down further on request).
  double approachSpeed_ = 0.14; // jab towards the card
  double retractSpeed_ = 0.09; // pull back from the card (kept slower than the jab)
  // Compression of the condition speed spread, applied around the BASELINE
  // (centre) scale: scale' = base + (scale - base) * compression. The centre
  // condition's tempo is untouched; Tool gets slower and Partner faster as the
  // spread narrows. 1 = full Tool/Baseline/Partner spread (as the handover),
  // 0 = all conditions at the Baseline tempo. Requested 2026-07-23 (two passes):
  // the full-spread Tool jab shook the bench, but compressing towards 1.0
  // dragged Partner too low — so compress towards the centre instead. With 0.3:
  // Tool x1.5 -> x1.22, Baseline x1.1 (unchanged), Partner x0.8 -> x1.01.
  // Monotonic, so the Tool > Baseline > Partner ordering is preserved.
  double speedScaleCompression_ = 0.3;
  // The final return home is done in JOINT space (not Cartesian) so the arm ends
  // at the HOME joint configuration, not merely the home EE position: a Cartesian
  // return can leave the joints in a twisted null-space config that Initial then
  // snaps to postureHome, tripping the e-stop (observed 2026-07-23, Baseline). It
  // always uses a gentle minimum-jerk profile, independent of the condition (the
  // return is not part of the NG cue), so it is smooth and slow across all three.
  double returnJointSpeed_ = 0.5; // [rad/s] peak joint velocity for the joint-space home return
  // Lower clamp on segment duration: only a guard against near-zero segments.
  // It must stay below the 0.10 m jab duration of the FASTEST condition
  // (~0.33 s at the current speeds), otherwise the clamp flattens (and can
  // even invert) the Tool/Baseline/Partner speed ordering on the most visible
  // part of the gesture — the jabs — which would break the condition
  // manipulation.
  double minMoveTime_ = 0.15; // [s]
  double convergenceGrace_ = 3.0; // [s] extra time allowed to converge on final poses

  // Gripper (requested 2026-07-23): NGPoint closes the gripper into a "pointing"
  // fist for the whole gesture and reopens it when done. Entry still requires an
  // open gripper (the refusal check in start()), so this can never squeeze a
  // held object. Set closeGripper_ false to keep the old open-hand pointing.
  bool closeGripper_ = true; // close the gripper for the gesture, reopen at the end
  double gripperPointOpening_ = 0.0; // target opening while pointing (0 = fully closed, 1 = fully open)
  double gripperSettle_ = 0.5; // [s] minimum wait before checking gripper completion

  // Trajectory style. Both are ALWAYS overwritten in start() from the
  // controller's active handover condition, so these initializers are only a
  // fallback for standalone use (same convention as BottlePick).
  double alpha_ = 1.0;
  double durationFactor_ = 1.4375; // common move-duration factor (all conditions), from the controller in start()
  double speedScale_ = 1.0;

  Eigen::Vector3d target_ = Eigen::Vector3d::Zero(); // card position being pointed at
  Eigen::Vector3d segStart_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d segEnd_ = Eigen::Vector3d::Zero();
  double t_ = 0.0;
  double tf_ = 0.0;
  double timer_ = 0.0; // [s] gripper close/open settle timer
  int jabsDone_ = 0;
  bool refused_ = false; // set when the gesture is refused (gripper not open)
  std::vector<double> startQ_ = {}; // joint-space return: start joint angles [rad]
  std::vector<double> targetQ_ = {}; // joint-space return: target (home) joint angles [rad]

  /** Start a straight-line EE segment to `goal`. If continuous, the segment
   *  starts from the previous segment's commanded end (hitch-free chaining, as
   *  BottlePick::initMove); otherwise from the current commanded position. */
  void initSegment(mc_control::fsm::Controller & ctl, const Eigen::Vector3d & goal, double speed, bool continuous);
  /** Advance the blended time scaling and feed position/refVel/refAccel to the
   *  EE task. Returns true when the segment is done: when the time is up for
   *  via-points, or once the task has converged for final poses. */
  bool segmentDone(mc_control::fsm::Controller & ctl, bool requireConvergence);
  /** Start a JOINT-space move to `poseDeg` (7 arm-joint angles in degrees, Kinova
   *  web-console convention) at peak joint velocity `speed` [rad/s]. Uses folding
   *  correction so each joint takes the short way (<=180 deg), and always a
   *  minimum-jerk profile. Used only for the safe return home. */
  void initJointMove(mc_control::fsm::Controller & ctl, const std::vector<double> & poseDeg, double speed);
  /** Advance the minimum-jerk joint-space return and feed the posture task.
   *  Returns true when the time is up and (if required) the joints have
   *  converged on the home configuration. */
  bool jointMoveDone(mc_control::fsm::Controller & ctl, bool requireConvergence);
  /** Resolve the pointed-at card position (explicit override or cardIndex). */
  Eigen::Vector3d cardTarget(mc_control::fsm::Controller & ctl) const;
  /** Offset point at `distance` in front of the card, along basedDirectionVec
   *  (same convention as the controller's initialize*Move helpers). */
  Eigen::Vector3d cardOffset(mc_control::fsm::Controller & ctl, double distance) const;
};
