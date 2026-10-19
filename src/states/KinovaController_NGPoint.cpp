#include "KinovaController_NGPoint.h"

#include "../KinovaController.h"

#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>

#include <cmath>
#include <map>

static const std::vector<std::string> jointNames = {"joint_1", "joint_2", "joint_3", "joint_4",
                                                    "joint_5", "joint_6", "joint_7"};

void KinovaController_NGPoint::configure(const mc_rtc::Configuration & config)
{
  config("cardIndex", cardIndex_);
  config("cardPosition", cardPosition_);
  config("startDistance", startDistance_);
  config("pointDistance", pointDistance_);
  config("retractDistance", retractDistance_);
  config("maxRepeats", maxRepeats_);
  config("prepSpeed", prepSpeed_);
  config("startSpeed", startSpeed_);
  config("approachSpeed", approachSpeed_);
  config("retractSpeed", retractSpeed_);
  config("returnJointSpeed", returnJointSpeed_);
  config("speedScaleCompression", speedScaleCompression_);
  config("minMoveTime", minMoveTime_);
  config("convergenceGrace", convergenceGrace_);
  config("closeGripper", closeGripper_);
  config("gripperPointOpening", gripperPointOpening_);
  config("gripperSettle", gripperSettle_);
}

void KinovaController_NGPoint::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);

  // Fail fast on a bad configuration, so a wrong YAML can never send the arm
  // through the card or make a segment duration negative/infinite (same
  // philosophy as the controller's handover-parameter validation).
  if(!cardPosition_.empty() && cardPosition_.size() != 3)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[NGPoint] cardPosition must have exactly 3 entries (got {})",
                                                     cardPosition_.size());
  }
  if(cardPosition_.empty() && (cardIndex_ < 1 || cardIndex_ > 3))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[NGPoint] cardIndex must be 1, 2 or 3 (got {})", cardIndex_);
  }
  if(!(pointDistance_ > 0.0 && retractDistance_ > pointDistance_ && startDistance_ >= retractDistance_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[NGPoint] distances must satisfy 0 < pointDistance < retractDistance <= startDistance (got {}, {}, {})",
        pointDistance_, retractDistance_, startDistance_);
  }
  for(double speed : {prepSpeed_, startSpeed_, approachSpeed_, retractSpeed_, returnJointSpeed_})
  {
    if(!(speed > 0.0))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[NGPoint] all speeds must be > 0 (got {})", speed);
    }
  }
  if(maxRepeats_ < 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[NGPoint] maxRepeats must be >= 0 (got {})", maxRepeats_);
  }
  if(!(minMoveTime_ > 0.0))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[NGPoint] minMoveTime must be > 0 (got {})", minMoveTime_);
  }
  if(!(speedScaleCompression_ >= 0.0 && speedScaleCompression_ <= 1.0))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[NGPoint] speedScaleCompression must be in [0, 1] (got {})",
                                                     speedScaleCompression_);
  }
  if(!(gripperPointOpening_ >= 0.0 && gripperPointOpening_ <= 1.0))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[NGPoint] gripperPointOpening must be in [0, 1] (got {})",
                                                     gripperPointOpening_);
  }
  if(!(gripperSettle_ >= 0.0))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[NGPoint] gripperSettle must be >= 0 (got {})", gripperSettle_);
  }
  if(cardPosition_.size() == 3)
  {
    // Sanity envelope for an explicit card override, so a YAML typo can never
    // aim the jab line at the participant, the robot itself or out of reach.
    // The default cardPosition_1..3 all sit inside this box.
    const double x = cardPosition_[0], y = cardPosition_[1], z = cardPosition_[2];
    if(!(x >= 0.3 && x <= 0.85 && std::abs(y) <= 0.4 && z >= 0.05 && z <= 0.7))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[NGPoint] cardPosition ({}, {}, {}) outside the allowed envelope x [0.3, 0.85], |y| <= 0.4, z [0.05, 0.7]",
          x, y, z);
    }
  }

  // Refuse to run while the gripper is holding something: NGPoint never touches
  // the gripper, so being entered mid-BottlePick (only possible via a Force
  // Transition) would swing a gripped bottle along the jab line near the
  // participant. Refusing outright is the safe answer; the normal NG flow
  // always starts from Initial with the gripper open.
  refused_ = false;
  if(ctl.robot().hasGripper("gripper") && ctl.robot().gripper("gripper").opening() < 0.9)
  {
    mc_rtc::log::warning("[NGPoint] Gripper is not open (opening {:.2f}) — refusing the pointing gesture "
                         "(was BottlePick interrupted?). Returning to Initial without moving.",
                         ctl.robot().gripper("gripper").opening());
    refused_ = true;
    phase_ = DONE;
    return;
  }

  // Adopt the active handover condition (motion style) for the whole gesture,
  // so the NG pointing and the bottle handover share one style. The speed spread
  // is compressed AROUND THE BASELINE scale (see speedScaleCompression_): the
  // centre tempo stays put, Tool comes down and Partner comes up, because the NG
  // jab doesn't need the handover's full tempo difference (full-spread Tool shook
  // the bench; 1.0-centred compression made Partner crawl).
  alpha_ = ctl.handoverAlpha();
  durationFactor_ = ctl.handoverDurationFactor();
  const double baseScale = ctl.handoverBaselineSpeedScale();
  speedScale_ = baseScale + (ctl.handoverSpeedScale() - baseScale) * speedScaleCompression_;
  mc_rtc::log::info("[NGPoint] Handover condition: {} (speed x{:.3f} = raw x{:.3f} compressed x{:.2f} about x{:.2f}, "
                    "alpha {:.2f})",
                    ctl.handoverConditionName(), speedScale_, ctl.handoverSpeedScale(), speedScaleCompression_,
                    baseScale, alpha_);

  target_ = cardTarget(ctl_);

  // Self-contained EE-task management: Initial (and a BottlePick run before it)
  // leaves compEETask out of the solver. Remove first so re-entry can never
  // double-add, reset so the task targets the CURRENT pose (no jump on entry),
  // then gear it like PrepareMovement did and add it back.
  ctl.solver().removeTask(ctl.compEETask);
  ctl.compEETask->reset();
  // reset() snaps the position/orientation targets to the current pose but
  // leaves any old feed-forward references in place; clear them explicitly.
  ctl.compEETask->positionTask->refVel(Eigen::Vector3d::Zero());
  ctl.compEETask->positionTask->refAccel(Eigen::Vector3d::Zero());
  ctl.compEETask->positionTask->stiffness(100.0);
  ctl.compEETask->positionTask->damping(50.0);
  ctl.compEETask->orientationTask->stiffness(100.0);
  ctl.compEETask->orientationTask->damping(50.0);
  // Pointing orientation: same fixed target (and same step-input convention)
  // as PrepareMovement; the 100/50 gains rotate the wrist there smoothly.
  ctl.compEETask->orientationTask->orientation(ctl.taskOrientation);
  ctl.solver().addTask(ctl.compEETask);
  // Keep the low-weight posture regularization anchored on home, as in the
  // original Initial -> PrepareMovement flow.
  ctl.compPostureTask->target(ctl.postureHome);

  jabsDone_ = 0;
  // Close the gripper into a pointing fist before moving. The EE task was just
  // reset() to the current pose, so the arm holds still while the gripper
  // closes; the entry move is started once CLOSE_GRIPPER completes. Skipped
  // (old open-hand pointing) when closeGripper_ is false or no gripper exists.
  if(closeGripper_ && ctl.robot().hasGripper("gripper"))
  {
    ctl.robot().gripper("gripper").setTargetOpening(gripperPointOpening_);
    timer_ = 0.0;
    phase_ = CLOSE_GRIPPER;
    mc_rtc::log::info("[NGPoint] Closing gripper to {:.0f}% before pointing", (1.0 - gripperPointOpening_) * 100.0);
  }
  else
  {
    initSegment(ctl_, ctl.homePosition, prepSpeed_, /*continuous=*/false);
    phase_ = MOVE_TO_HOME;
    mc_rtc::log::info("[NGPoint] Moving to EE home position ({:.1f} s)", tf_);
  }
}

void KinovaController_NGPoint::initSegment(mc_control::fsm::Controller & ctl_,
                                           const Eigen::Vector3d & goal,
                                           double speed,
                                           bool continuous)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  // For a continuous chain, start from the previous segment's commanded end so
  // the position command stays continuous through via-points (no hitch); the
  // first segment starts from the currently commanded position instead.
  segStart_ = continuous ? segEnd_ : Eigen::Vector3d(ctl.compEETask->positionTask->position());
  segEnd_ = goal;
  const double dist = (segEnd_ - segStart_).norm();
  // speed means peak velocity for every alpha: the blended profile's peak/mean
  // velocity ratio is (1 + 0.875*alpha), exactly as in BottlePick::initMove.
  // Same duration in every condition (alpha only shapes the profile).
  const double peakFactor = durationFactor_;
  tf_ = std::max(peakFactor * dist / (speed * speedScale_), minMoveTime_);
  t_ = 0.0;
}

bool KinovaController_NGPoint::segmentDone(mc_control::fsm::Controller & ctl_, bool requireConvergence)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  t_ += ctl.timeStep;
  const double u = std::min(t_ / tf_, 1.0);
  const Eigen::Vector3d delta = segEnd_ - segStart_;
  if(t_ < tf_)
  {
    // Blended time scaling s_alpha(u) = (1-alpha)*u + alpha*(10u^3-15u^4+6u^5):
    // alpha 0 -> constant velocity (sharp start/stop), alpha 1 -> minimum jerk.
    // Same profile as BottlePick, applied to a straight Cartesian line; the
    // velocity/acceleration references keep the EE task tracking tight, like
    // the controller's minimumJerkMove/constantVelocityMove did.
    const double mj = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    const double s = (1.0 - alpha_) * u + alpha_ * mj;
    const double dsdu = (1.0 - alpha_) + alpha_ * 30.0 * u * u * (1.0 - u) * (1.0 - u);
    const double d2sdu2 = alpha_ * (60.0 * u - 180.0 * u * u + 120.0 * u * u * u);
    ctl.compEETask->positionTask->position(segStart_ + s * delta);
    ctl.compEETask->positionTask->refVel(delta * (dsdu / tf_));
    ctl.compEETask->positionTask->refAccel(delta * (d2sdu2 / (tf_ * tf_)));
    return false;
  }
  // Time is up: pin the command on the endpoint with zero feed-forward. For
  // alpha < 1 the reference velocity was non-zero right up to tf; cutting it
  // here IS the abrupt-stop cue of the Tool condition. Precedent: the original
  // ConstantVel_FSM card-pointing mode does exactly this (feed-forward velocity
  // held at vMax until tf, then zeroed in one tick) and is verified on the
  // real robot; BottlePick's cue is position-only (no feed-forward there).
  ctl.compEETask->positionTask->position(segEnd_);
  ctl.compEETask->positionTask->refVel(Eigen::Vector3d::Zero());
  ctl.compEETask->positionTask->refAccel(Eigen::Vector3d::Zero());
  if(!requireConvergence)
  {
    return true;
  }
  if(ctl.compEETask->eval().norm() < 0.01)
  {
    return true;
  }
  if(t_ > tf_ + convergenceGrace_)
  {
    mc_rtc::log::warning("[NGPoint] Segment did not fully converge (task error {:.3f}), continuing",
                         ctl.compEETask->eval().norm());
    return true;
  }
  return false;
}

Eigen::Vector3d KinovaController_NGPoint::cardTarget(mc_control::fsm::Controller & ctl_) const
{
  const auto & ctl = static_cast<const KinovaController &>(ctl_);
  if(cardPosition_.size() == 3)
  {
    return Eigen::Vector3d(cardPosition_[0], cardPosition_[1], cardPosition_[2]);
  }
  switch(cardIndex_)
  {
    case 1:
      return ctl.cardPosition_1;
    case 2:
      return ctl.cardPosition_2;
    default:
      return ctl.cardPosition_3;
  }
}

Eigen::Vector3d KinovaController_NGPoint::cardOffset(mc_control::fsm::Controller & ctl_, double distance) const
{
  const auto & ctl = static_cast<const KinovaController &>(ctl_);
  // Same offset convention as the controller's initialize*Move helpers: the
  // approach line runs along basedDirectionVec towards the card.
  return target_ - distance * ctl.basedDirectionVec;
}

void KinovaController_NGPoint::initJointMove(mc_control::fsm::Controller & ctl_,
                                             const std::vector<double> & poseDeg,
                                             double speed)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  constexpr double TWO_PI = 2.0 * mc_rtc::constants::PI;
  startQ_.clear();
  targetQ_.clear();
  double maxDelta = 0.0;
  for(size_t i = 0; i < jointNames.size(); ++i)
  {
    // Start from the actual joint value (this is the first, and only, joint-space
    // move of the state), so the posture target begins at the current pose.
    const double ref = ctl.robot().mbc().q[ctl.robot().jointIndexByName(jointNames[i])][0];
    // Kinova web-console angles live in [0, 360); fold to the equivalent angle
    // closest to the current reference so no joint takes the long way around
    // (same correction as BottlePick::initMove — this is what avoids the e-stop).
    double target = mc_rtc::constants::toRad(poseDeg[i]);
    target += TWO_PI * std::round((ref - target) / TWO_PI);
    startQ_.push_back(ref);
    targetQ_.push_back(target);
    maxDelta = std::max(maxDelta, std::abs(target - ref));
  }
  // Always minimum jerk (peak/mean velocity ratio 1.875) for a smooth return that
  // eases to zero velocity, independent of the handover condition. `speed` is the
  // peak joint velocity [rad/s]; minMoveTime_ guards against a near-zero move.
  const double peakFactor = 1.875;
  tf_ = std::max(peakFactor * maxDelta / speed, minMoveTime_);
  t_ = 0.0;
}

bool KinovaController_NGPoint::jointMoveDone(mc_control::fsm::Controller & ctl_, bool requireConvergence)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  t_ += ctl.timeStep;
  const double u = std::min(t_ / tf_, 1.0);
  const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u)); // minimum-jerk time scaling
  std::map<std::string, std::vector<double>> target;
  double maxErr = 0.0;
  for(size_t i = 0; i < jointNames.size(); ++i)
  {
    target[jointNames[i]] = {startQ_[i] + s * (targetQ_[i] - startQ_[i])};
    const double q = ctl.robot().mbc().q[ctl.robot().jointIndexByName(jointNames[i])][0];
    maxErr = std::max(maxErr, std::abs(q - targetQ_[i]));
  }
  ctl.compPostureTask->target(target);
  if(t_ < tf_)
  {
    return false;
  }
  if(!requireConvergence)
  {
    return true;
  }
  if(maxErr < 0.05)
  {
    return true;
  }
  if(t_ > tf_ + convergenceGrace_)
  {
    mc_rtc::log::warning("[NGPoint] Joint return did not fully converge (max joint error {:.3f} rad), continuing",
                         maxErr);
    return true;
  }
  return false;
}

bool KinovaController_NGPoint::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  switch(phase_)
  {
    case CLOSE_GRIPPER:
      // Arm holds its pose (EE task target unchanged) while the gripper closes.
      // Same completion pattern as BottlePick: wait a minimum settle, then for
      // complete(), with a hard timeout so a non-reporting gripper cannot stall.
      timer_ += ctl.timeStep;
      if((timer_ > gripperSettle_ && ctl.robot().gripper("gripper").complete()) || timer_ > 4.0)
      {
        if(timer_ > 4.0)
        {
          mc_rtc::log::warning("[NGPoint] Gripper close did not report completion, continuing anyway");
        }
        initSegment(ctl_, ctl.homePosition, prepSpeed_, /*continuous=*/false);
        phase_ = MOVE_TO_HOME;
        mc_rtc::log::info("[NGPoint] Gripper closed, moving to EE home position ({:.1f} s)", tf_);
      }
      break;
    case MOVE_TO_HOME:
      // Via-point: flow straight into the start waypoint
      if(segmentDone(ctl_, /*requireConvergence=*/false))
      {
        initSegment(ctl_, cardOffset(ctl_, startDistance_), startSpeed_, /*continuous=*/true);
        phase_ = MOVE_TO_START;
        mc_rtc::log::info("[NGPoint] Moving to start waypoint ({:.1f} s)", tf_);
      }
      break;
    case MOVE_TO_START:
      if(segmentDone(ctl_, /*requireConvergence=*/false))
      {
        initSegment(ctl_, cardOffset(ctl_, pointDistance_), approachSpeed_, /*continuous=*/true);
        phase_ = POINT;
        mc_rtc::log::info("[NGPoint] Pointing at the card ({} jabs)", 1 + maxRepeats_);
      }
      break;
    case POINT:
      if(segmentDone(ctl_, /*requireConvergence=*/false))
      {
        ++jabsDone_;
        if(jabsDone_ <= maxRepeats_)
        {
          initSegment(ctl_, cardOffset(ctl_, retractDistance_), retractSpeed_, /*continuous=*/true);
          phase_ = RETRACT;
        }
        else
        {
          // Return home in JOINT space, not Cartesian: hand the arm to Initial
          // already sitting at the home joint configuration so its one-shot
          // posture-home command has nothing to snap (a Cartesian return can
          // leave the joints twisted and trip the e-stop). Drop the EE task so
          // the posture task drives the joints; the move starts from the current
          // q (no jump) and eases to home with a gentle minimum-jerk profile.
          ctl.solver().removeTask(ctl.compEETask);
          std::vector<double> homeDeg;
          for(const auto & jn : jointNames)
          {
            homeDeg.push_back(ctl.postureHome.at(jn)[0] * 180.0 / mc_rtc::constants::PI);
          }
          initJointMove(ctl_, homeDeg, returnJointSpeed_);
          phase_ = RETURN_TO_HOME;
          mc_rtc::log::info("[NGPoint] Gesture done, returning to home joint configuration ({:.1f} s)", tf_);
        }
      }
      break;
    case RETRACT:
      if(segmentDone(ctl_, /*requireConvergence=*/false))
      {
        initSegment(ctl_, cardOffset(ctl_, pointDistance_), approachSpeed_, /*continuous=*/true);
        phase_ = POINT;
      }
      break;
    case RETURN_TO_HOME:
      if(jointMoveDone(ctl_, /*requireConvergence=*/true))
      {
        mc_rtc::log::info("[NGPoint] Returned to home joint configuration.");
        // Reopen the gripper (leaving it open for the next BottlePick, which
        // starts by opening anyway). Only if we closed it in the first place.
        if(closeGripper_ && ctl.robot().hasGripper("gripper"))
        {
          ctl.robot().gripper("gripper").setTargetOpening(1.0);
          timer_ = 0.0;
          phase_ = OPEN_GRIPPER;
          mc_rtc::log::info("[NGPoint] Reopening gripper");
        }
        else
        {
          phase_ = DONE;
        }
      }
      break;
    case OPEN_GRIPPER:
      timer_ += ctl.timeStep;
      if((timer_ > gripperSettle_ && ctl.robot().gripper("gripper").complete()) || timer_ > 2.0)
      {
        if(timer_ > 2.0)
        {
          mc_rtc::log::warning("[NGPoint] Gripper open did not report completion, continuing anyway");
        }
        phase_ = DONE;
      }
      break;
    case DONE:
      output("OK");
      return true;
  }
  return false;
}

void KinovaController_NGPoint::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  // Leave the solver in the same shape Initial expects (no EE task); Initial's
  // own removeTask call then becomes a harmless no-op.
  ctl.solver().removeTask(ctl.compEETask);
}

EXPORT_SINGLE_STATE("KinovaController_NGPoint", KinovaController_NGPoint)
