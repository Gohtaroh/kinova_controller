#pragma once

#include <mc_control/fsm/State.h>

#include <RBDyn/MultiBodyConfig.h>

#include <string>
#include <vector>

#include "RunLogger.h"

/** One-button bottle pick, POSITION-specified (prototype, 2026-07-23).
 *
 *  Same behaviour as KinovaController_BottlePick — approach above the bottle,
 *  top-down grasp, lift straight up, present to the participant, hold with
 *  pull-to-release detection, then a safe folding-corrected return home — but
 *  the pick is defined by CARTESIAN targets (x, y, z) instead of taught joint
 *  angles. The approach / grasp / lift points and the grasp orientation are, by
 *  default, computed by forward kinematics from the same taught poses BottlePick
 *  uses, so out of the box Bottle1 reproduces the current motion ("実測"); each
 *  point can then be overridden in metres from the FSM config to move the pick
 *  (the basis for laying 4 bottles in a row: common x/z, different y).
 *
 *  Front half (approach/grasp/lift) runs on the compliant EE task with the same
 *  blended time scaling as NGPoint/BottlePick; the back half (present/hold/
 *  release/return) is joint-space and copied from BottlePick unchanged, so the
 *  proven pull-detection and safe return are reused verbatim. The EE task is
 *  dropped at the lift->present handoff, so the arm ends at the home joint
 *  configuration and Initial has nothing to snap (auto-return is safe).
 *
 *  Per-run CSV (2026-10, early-release study): every execution writes ONE file
 *  <runLogDir>/<YYYYMMDD_HHMMSS>_<StateName>_<Condition>.csv holding, at the
 *  control rate, the joint state, measured and external-estimate torques of
 *  all 7 joints, end-effector position/velocity, the distance left to the
 *  present pose, and the pull-detection internals, in EVERY phase. Metadata
 *  (condition, parameters, resolved pick points) and events (phase changes,
 *  detection) are '#' header lines. See RunLogger.h.
 */
struct KinovaController_Bottle1 : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  enum Phase
  {
    MOVE_TO_APPROACH, // Cartesian: home -> waypoint above the bottle
    MOVE_TO_GRASP, // Cartesian: descend straight down onto the bottle
    CLOSE_GRIPPER, // close the gripper on the bottle
    MOVE_TO_LIFTUP, // Cartesian: lift straight up off the table
    MOVE_TO_PRESENT, // joint-space: swing to the common present/handover pose
    HOLDING, // hold and watch for the participant's pull (joint-space)
    RELEASING, // pull detected / button: open the gripper
    RETURN_TO_HOME, // joint-space folding-corrected return home
    DONE
  };
  Phase phase_ = MOVE_TO_APPROACH;

  // --- Pick geometry -------------------------------------------------------
  // Measurement source: the taught BottlePick poses (Kinova web-console degrees,
  // [0, 360)). At start() their EE Cartesian pose is obtained by forward
  // kinematics; approach/grasp/lift points default to those FK positions and the
  // grasp orientation to the FK orientation of poseA_deg (top-down grasp).
  std::vector<double> poseApproachDeg_ = {359.707, 59.4, 180.031, 214.838, 359.63, 114.512, 88.379};
  std::vector<double> poseADeg_ = {359.501, 81.853, 180.238, 238.467, 359.126, 113.316, 88.062};
  std::vector<double> poseLiftDeg_ = {359.72, 52.925, 180.193, 224.675, 359.867, 98.206, 88.765};
  // Optional Cartesian overrides [m], size 3 each. Empty => use the FK value.
  // This is the "specify by position" interface (and the hook for Bottle2..4).
  std::vector<double> approachPos_ = {};
  std::vector<double> graspPos_ = {};
  std::vector<double> liftPos_ = {};
  // Lateral shift [m] added to the whole pick column (+y = robot's left), applied
  // on top of the FK/override points. This is how Bottle1..4 sit side by side: same
  // x, z and grasp orientation as the measured pick, only y differs. x/z stay
  // common so all bottles are grasped top-down at one height, as requested.
  double lateralOffset_ = 0.0;

  // Present/handover pose stays joint-space and common to every bottle (same
  // default as BottlePick): the pull-detection baseline/thresholds are tuned for
  // this exact pose, so it must not change per bottle.
  std::vector<double> poseBDeg_ = {1.451, 88.829, 188.223, 358.957, 356.36, 0.63, 85.62};

  double gripperClosedPct_ = 43.0; // [%] Kinova convention: 0 = open, 100 = closed

  // Peak Cartesian speeds [m/s] for the pick (before the condition speed scale).
  // Slightly lowered 2026-07-23 (x~0.8 from the first real-robot values): the
  // Tool condition was brisk enough to nudge the bench/bottles, so the whole
  // pick runs a touch slower across all conditions.
  double approachSpeed_ = 0.16; // home -> above the bottle
  double graspSpeed_ = 0.08; // gentle straight-down descent onto the bottle
  double liftSpeed_ = 0.12; // straight-up lift off the table
  // Peak joint speeds [rad/s] for the joint-space back half (BottlePick's 0.8,
  // lowered with the rest).
  double presentSpeed_ = 0.65; // lift point -> present pose
  double returnSpeed_ = 0.65; // present pose -> home

  // --- Pull-to-release detection at the present pose (copied from BottlePick) -
  double pullJoint4Thresh_ = 0.5;
  double pullJoint2Thresh_ = 0.5;
  double pullJoint4Abs_ = 2.0;
  double pullJoint2Abs_ = 2.5;
  double earlyPullJoint4Abs_ = 2.3;
  double earlyPullJoint2Abs_ = 2.8;
  double earlyDebounceTime_ = 0.35;
  double armDelay_ = 0.2; // overwritten per-condition in start()
  double debounceTime_ = 0.15;
  double gripperSettle_ = 0.5;
  double convergenceGrace_ = 3.0;
  double minMoveTime_ = 0.5; // [s] lower clamp on any segment duration
  // Simulation aid: if > 0, release automatically after this many seconds of
  // HOLDING (there is no one to pull in MuJoCo / the ticker). 0 = disabled,
  // which is what the real robot must run with.
  double autoReleaseAfter_ = 0.0;

  // --- Per-run CSV log --------------------------------------------------------
  bool runLogEnabled_ = true;
  std::string runLogDir_; // default: $HOME/workspace/logs/runs (set in configure)
  int runLogDecimation_ = 1; // 1 = every control tick
  double runLogReserveSec_ = 120.0; // buffer preallocation (longer runs still work, with one realloc)
  RunLogger runLog_;
  std::vector<double> rowBuf_;
  std::vector<int> refTorqueIdx_; // per arm joint: index into realRobot().jointTorques(), or -1
  std::vector<int> jointIdx_; // per arm joint: mb joint index
  std::vector<int> dofIdx_; // per arm joint: DoF index (external-torque vector)
  Eigen::VectorXd tauExt_ = Eigen::VectorXd::Zero(0); // this tick's full estimate (all DoFs)
  double runTime_ = 0.0; // [s] since start()
  int tick_ = 0;
  int releaseTrigger_ = 0; // 0 none, 1 pull (delta/abs), 2 early pull, 3 manual button, 4 auto (sim)
  double segProgress_ = 0.0; // t/tf of the current segment, clamped to [0, 1]
  /** Record a line in the mc_rtc terminal log AND as a run-log event. */
  void note(const std::string & text);
  /** Before-stop pull check (window entry, MA vs reference, debounce). Returns true to release now. */
  bool earlyPullCheck(mc_control::fsm::Controller & ctl);
  /** Append this tick's sample row to the run log. */
  void logRow(mc_control::fsm::Controller & ctl);

  // --- Trajectory style (read from the active handover condition in start()) -
  double alpha_ = 1.0;
  double durationFactor_ = 1.4375; // common move-duration factor (all conditions), from the controller in start()
  double speedScale_ = 1.0;

  // --- Computed pick targets (filled in start() from config or FK) ----------
  Eigen::Vector3d approachPt_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d graspPt_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d liftPt_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d graspOri_ = Eigen::Matrix3d::Identity(); // held over approach/grasp/lift

  // --- Cartesian segment state (EE task) ------------------------------------
  Eigen::Vector3d segStart_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d segEnd_ = Eigen::Vector3d::Zero();
  // --- Joint move state (posture task) --------------------------------------
  std::vector<double> startQ_;
  std::vector<double> targetQ_;

  double t_ = 0.0;
  double tf_ = 0.0;
  double timer_ = 0.0;
  bool returnRequested_ = false; // GUI "Return home (safe)" button

  // Pull-detection runtime state (copied from BottlePick)
  double holdTimer_ = 0.0;
  double debounceTimer_ = 0.0;
  double earlyDebounceTimer_ = 0.0;
  int dofJoint2_ = -1;
  int dofJoint4_ = -1;
  bool forceDetectAvailable_ = false;
  bool baselineCaptured_ = false;
  // The resting baseline is the MEAN over baselineWindow_ seconds after the arm
  // delay, not one sample: after the stop the arm rings at ~5-6 Hz, and a single
  // sample taken on a ringing peak made the detector release with no pull
  // (2026-10-19 run analysis). 0.3 s covers more than one ringing period.
  double baselineWindow_ = 0.3; // [s]
  double baselineSum2_ = 0.0;
  double baselineSum4_ = 0.0;
  int baselineN_ = 0;
  double baselineTimer_ = 0.0;
  // Second release path (2026-10-19, run analysis): a pull makes the arm ring at
  // ~6 Hz, so the instantaneous deviation keeps dipping back under its threshold
  // and the strict 0.15 s debounce can take >1 s. This path averages
  // (tau_ext_2 - tau_ext_4) over one ringing period (pullMaWindow_), which
  // cancels the ringing, and compares it with the same quantity's resting
  // baseline. Either path releases. Validated on the 2026-10-19 runs: no early
  // release, the slow run would have released 1.5 s sooner.
  double pullMaWindow_ = 0.18; // [s]
  double pullMaThresh_ = 1.25; // [Nm] on MA(tau2 - tau4) - baseline (positive = pull)
  double pullMaDebounce_ = 0.05; // [s]
  std::vector<double> maBuf_;
  size_t maIdx_ = 0;
  size_t maCount_ = 0;
  double maSum_ = 0.0;
  double lastMa_ = 0.0; // MA(tau2 - tau4) - baseline, logged
  double maDebounceTimer_ = 0.0;
  // The moving-average path is armed soon after the stop, independent of the
  // per-condition armDelay (which protects the ringing-sensitive delta path):
  // its own baseline is the mean of (tau2 - tau4) over baselineWindow_ starting
  // pullMaArmDelay_ after the stop. Replay of the 20 runs of 2026-10-19: no
  // release before the pull; Tool pulls 1-2.6 s after the stop released in
  // ~0.15 s instead of ~1 s.
  double pullMaArmDelay_ = 0.2; // [s]
  double maBaseSum_ = 0.0;
  int maBaseN_ = 0;
  double maBaseline_ = 0.0;
  bool maArmed_ = false;
  double lastRawMa_ = 0.0; // this tick's MA(tau2 - tau4), computed every tick in every phase
  // Before-stop ("early") pull release, 2026-10-19 (analysis report Plan A):
  // monitoring starts when the EE is within earlyWindowDist_ of the present pose;
  // reference = mean of (tau2 - tau4) over the earlyRefWindow_ seconds just before
  // entering the window; release when MA - reference > pullMaThresh_ for
  // pullMaDebounce_. Stays active through HOLDING until the hold MA path arms.
  // On/off from the controller GUI; when off, would-be releases are logged only.
  double earlyWindowDist_ = 0.15; // [m]
  double earlyRefWindow_ = 0.5; // [s]
  std::vector<double> refBuf_;
  size_t refIdx_ = 0;
  size_t refCount_ = 0;
  double refSum_ = 0.0;
  bool earlyArmed_ = false;
  double earlyRef_ = 0.0;
  double lastEarlyMa_ = 0.0;
  double earlyMaDebounce_ = 0.0;
  bool earlyShadowLogged_ = false;
  bool releaseFromCurrent_ = false; // released mid-motion: return home from the current pose
  double baselineJoint2_ = 0.0;
  double baselineJoint4_ = 0.0;
  double lastTau2_ = 0.0;
  double lastTau4_ = 0.0;
  double lastD2_ = 0.0;
  double lastD4_ = 0.0;
  // Diagnostics for the "release while still reaching out" study (2026-10): log
  // the external torque in EVERY phase (not just HOLDING) plus the current phase
  // and how far the end-effector still is from the present pose, so the motion-
  // phase external-torque profile (and how a pull differs from it) is plottable.
  Eigen::Vector3d presentEEPos_ = Eigen::Vector3d::Zero(); // FK of poseB (present), world frame
  double lastDistToPresent_ = 0.0; // [m] live EE distance to the present pose, all phases

  /** Forward-kinematics EE pose (end_effector_link, world frame) at a taught
   *  joint pose, WITHOUT disturbing the live robot state (works on a temp mbc). */
  sva::PTransformd poseFK(mc_control::fsm::Controller & ctl, const std::vector<double> & poseDeg) const;
  /** Resolve a pick point: the config override if given, else the FK translation. */
  static Eigen::Vector3d resolvePoint(const std::vector<double> & override3, const sva::PTransformd & fk);

  /** Start / advance a straight-line Cartesian EE segment (blended s_alpha, same
   *  profile as NGPoint). segmentDone returns true when the time is up (via) or
   *  once the task has converged (final). */
  void initSegment(mc_control::fsm::Controller & ctl, const Eigen::Vector3d & goal, double speed, bool continuous);
  bool segmentDone(mc_control::fsm::Controller & ctl, bool requireConvergence);

  /** Start / advance a joint-space minimum-jerk-blended posture move (folding
   *  corrected, same as BottlePick::initMove/moveDone). Used for present + return. */
  void initMove(mc_control::fsm::Controller & ctl, const std::vector<double> & poseDeg, double speed, bool continuous);
  bool moveDone(mc_control::fsm::Controller & ctl, bool requireConvergence);
};
