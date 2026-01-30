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

  void updateTasksRef();

  void initializeConstantVelocityMove(Eigen::Vector3d target, double distance_from_target, double v_max);
  void constantVelocityMove();

  void initializeTrapezoidalVelocityMove(Eigen::Vector3d target, double distance_from_target, double v_max, double a_max);
  void trapezoidalVelocityMove();

  void initializeMinimumJerkMove(Eigen::Vector3d target, double distance_from_target, double v_max);
  void minimumJerkMove();

  bool motionComplete() const;

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

  Eigen::Vector3d basedDirectionVec; // Direction vector used to define the new target position at a distance from the card
  Eigen::Vector3d directionVec; // Current direction vector for linear movement

private:
  mc_rtc::Configuration config_;
  
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
