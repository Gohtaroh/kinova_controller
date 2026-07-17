#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>

#include "api.h"

struct KinovaController_DLLAPI KinovaController : public mc_control::fsm::Controller
{
  KinovaController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void updateTasksRef();

  void initializeConstantVelocityMove(Eigen::Vector3d target, double distance_from_target, double v_max);
  void constantVelocityMove();

  void initializeTrapezoidalVelocityMove(Eigen::Vector3d target,
                                         double distance_from_target,
                                         double v_max,
                                         double a_max);
  void trapezoidalVelocityMove();

  void initializeMinimumJerkMove(Eigen::Vector3d target, double distance_from_target, double v_max);
  void minimumJerkMove();

  bool motionComplete() const;

  // Active handover motion-style condition (personalized-collaboration study).
  // Each condition combines a speed scale (multiplies the BottlePick move speeds)
  // with a trajectory blend alpha: 0 = constant velocity (mechanical), 1 = minimum
  // jerk (human-like). The two axes are bundled and moved together so the three
  // conditions are perceptually distinguishable. Selected from the GUI dropdown;
  // BottlePick reads these at start() so the whole sequence takes one style.
  double handoverSpeedScale() const
  {
    return handoverSpeedScales_[handoverCondition_];
  }
  double handoverAlpha() const
  {
    return handoverAlphas_[handoverCondition_];
  }
  // Per-condition settle time before pull detection arms. A sharper stop (alpha < 1)
  // leaves a larger external-torque transient, so the abrupt conditions need to wait
  // longer before the resting baseline is captured, otherwise the settling transient
  // is misread as a pull and the bottle is released on its own.
  double handoverArmDelay() const
  {
    return handoverArmDelays_[handoverCondition_];
  }
  const std::string & handoverConditionName() const
  {
    return handoverConditionNames_[handoverCondition_];
  }

  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> compEETask;

  // Targets
  std::map<std::string, std::vector<double>> postureHome;

  // EE task variables
  Eigen::MatrixXd taskOrientation; // Rotation Matrix
  Eigen::Vector3d taskPosition;
  Eigen::Vector3d taskVelocity;
  Eigen::Vector3d taskAcceleration;
  Eigen::MatrixXd homeOrientation; // Rotation Matrix
  Eigen::Vector3d homePosition;

  // EE Target positions
  Eigen::Vector3d cardPosition_1;
  Eigen::Vector3d cardPosition_2;
  Eigen::Vector3d cardPosition_3;

  Eigen::Vector3d
      basedDirectionVec; // Direction vector used to define the new target position at a distance from the card
  Eigen::Vector3d directionVec; // Current direction vector for linear movement

private:
  mc_rtc::Configuration config_;

  // Handover conditions: index 0 = tool (mechanical), 1 = baseline, 2 = partner.
  // Speed scales are geometric about the baseline (baseline = geometric mean of tool
  // and partner, ratio ~1.37), so the middle condition stays the speed-midpoint of
  // the two extremes; alphas span constant velocity -> minimum jerk. Arm delays grow
  // as alpha drops (sharper stop -> larger torque transient -> longer settle needed).
  // All placeholders to be confirmed by the 7/24 pilot; overridable via YAML config.
  std::vector<std::string> handoverConditionNames_ = {"Tool", "Baseline", "Partner"};
  std::vector<double> handoverSpeedScales_ = {1.5, 1.1, 0.8};
  std::vector<double> handoverAlphas_ = {0.0, 0.5, 1.0};
  std::vector<double> handoverArmDelays_ = {3.0, 2.0, 0.2}; // [s] settle before pull detection arms
  size_t handoverCondition_ = 1; // default: Baseline

  Eigen::Vector3d initialEEPosition_;
  Eigen::Vector3d finalEEPosition_;

  double currentTargetDistance_ = 0.0;
  double currentTargetVelocity_ = 0.0;
  double currentTargetAcceleration_ = 0.0;

  double t_ = 0.0; // Time variable for motion profiling
  double tf_ = 0.0; // Total time for the motion

  // Trapezoidal profile parameters
  double t1_ = 0.0; // Time to reach v_max
  double t2_ = 0.0; // Time at which deceleration starts
  double d1_ = 0.0; // Distance covered during acceleration phase
  double d2_ = 0.0; // Distance covered during constant velocity phase

  double distanceEECard1_ = 0.0;
  double distanceEECard2_ = 0.0;
  double distanceEECard3_ = 0.0;

  double totalDistance_ = 0.0;
  double vMax_ = 0.0;
  double aMax_ = 0.0;

  std::string frame_ = "end_effector_link";
};
