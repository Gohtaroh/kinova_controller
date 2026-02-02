#include "KinovaController.h"
#include <mc_rtc/logging.h>
#include <cmath>

KinovaController::KinovaController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  frame_ = config("frame", (std::string) "end_effector_link");

  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.1, 9.0});
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(
          robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  postureHome = {{"joint_1", {0}},    {"joint_2", {0.262}},
                 {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                 {"joint_5", {0}},    {"joint_6", {0.96}},
                 {"joint_7", {1.57}}};

  // Create the compliant posture task
  compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(
      solver(), robot().robotIndex(), 100.0, 1.0);
  solver().addTask(compPostureTask);

  // Create the compliant end-effector task
  compEETask = std::make_shared<mc_tasks::CompliantEndEffectorTask>(
      frame_, robots(), robot().robotIndex(), 100.0, 1000.0);

  compEETask->reset();
  solver().addTask(compEETask);


  homeOrientation =
      Eigen::Quaterniond(1, -1, -1, -1).normalized().toRotationMatrix(); // Based orientation
  taskOrientation =
      Eigen::Quaterniond(-0.271, 0.653, 0.653, 0.271).normalized().toRotationMatrix();

  homePosition = compEETask->positionTask->position();
  taskPosition = homePosition;
  taskVelocity = Eigen::Vector3d::Zero();
  taskAcceleration = Eigen::Vector3d::Zero();

  // Define EE target positions
  cardPosition_1 = Eigen::Vector3d(0.7, 0.0, 0.2);
  cardPosition_2 = Eigen::Vector3d(0.7, -0.2, 0.2);
  cardPosition_3 = Eigen::Vector3d(0.7, 0.2, 0.2);

  // Set control mode in datastore
  datastore().make<std::string>("TorqueMode", "Custom");
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });
  
  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));

  basedDirectionVec = Eigen::Vector3d(1, 0, -1).normalized(); // Unit vector along (1,1,-1) direction at 45 degrees

  directionVec = basedDirectionVec;
  initialEEPosition_ = taskPosition;
  finalEEPosition_ = taskPosition;

  // Add GUI 
  gui()->addElement(
      {"KinovaController", "Movement"},
      mc_rtc::gui::ArrayInput("Direction Vector", {"x", "y", "z"},
        [this]() { return directionVec; },
          [this](const Eigen::Vector3d & dir) {directionVec = dir;}
        )
      );

  
  gui()->addElement(
      {"KinovaController", "Movement"},
      mc_rtc::gui::Point3DRO("Final Position", finalEEPosition_)
      );
  gui()->addElement(
      {"KinovaController", "Card Positions"},
      mc_rtc::gui::Point3DRO("Card Position 1", cardPosition_1)
      );
  gui()->addElement(
      {"KinovaController", "Card Positions"},
      mc_rtc::gui::Point3DRO("Card Position 2", cardPosition_2)
      );
  gui()->addElement(
      {"KinovaController", "Card Positions"},
      mc_rtc::gui::Point3DRO("Card Position 3", cardPosition_3)
      );

  // Add logging
  logger().addLogEntry("KinovaController_DirectionVec", [this]() { return directionVec; });
  logger().addLogEntry("KinovaController_DirectionVec_norm", [this]() { return directionVec.norm(); });
  logger().addLogEntry("KinovaController_TargetDistance", [this]() { return currentTargetDistance_; });
  logger().addLogEntry("KinovaController_TargetVelocity", [this]() { return currentTargetVelocity_; });
  logger().addLogEntry("KinovaController_TargetAcceleration", [this]() { return currentTargetAcceleration_; });
  logger().addLogEntry("KinovaController_DistanceEECard1", [this]() { return distanceEECard1_; });
  logger().addLogEntry("KinovaController_DistanceEECard2", [this]() { return distanceEECard2_; });
  logger().addLogEntry("KinovaController_DistanceEECard3", [this]() { return distanceEECard3_; });
  logger().addLogEntry("KinovaController_FinalEEPosition", [this]() { return finalEEPosition_; });
  logger().addLogEntry("KinovaController_InitialEEPosition", [this]() { return initialEEPosition_; });
  logger().addLogEntry("KinovaController_TotalDistance", [this]() { return totalDistance_; });
  logger().addLogEntry("KinovaController_vMax", [this]() { return vMax_; });
  logger().addLogEntry("KinovaController_aMax", [this]() { return aMax_; });
  logger().addLogEntry("KinovaController_Time", [this]() { return t_; });
  logger().addLogEntry("KinovaController_TotalTime", [this]() { return tf_; });
  logger().addLogEntry("KinovaController_PlannerDistance", [this]() { return (finalEEPosition_ - cardPosition_1).norm(); });



  mc_rtc::log::success("KinovaController init done ");
}

bool KinovaController::run()
{
  distanceEECard1_ = (compEETask->positionTask->position() - cardPosition_1).norm();
  distanceEECard2_ = (compEETask->positionTask->position() - cardPosition_2).norm();
  distanceEECard3_ = (compEETask->positionTask->position() - cardPosition_3).norm();

   // Update the solver depending on the control mode
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if (ctrl_mode.compare("Position") == 0) {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  } 
  // else Torque control mode (Closed loop)
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
}

void KinovaController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void KinovaController::initializeConstantVelocityMove(Eigen::Vector3d target, double distance_from_target, double v_max)
{
  finalEEPosition_ = target - distance_from_target * basedDirectionVec;
  initialEEPosition_ = compEETask->positionTask->position();
  Eigen::Vector3d distance_from_EE = (finalEEPosition_ - initialEEPosition_);
  directionVec = (distance_from_EE).normalized();
  totalDistance_ = distance_from_EE.norm();
  vMax_ = v_max;
  tf_ = totalDistance_ / vMax_;
  t_  = 0.0;
  mc_rtc::log::info("[KinovaController] Constant velocity move initialized. Total time: {:.2f} s", tf_);
}

void KinovaController::constantVelocityMove()
{
  t_ += timeStep;

  if (t_ >= tf_)
  {
    mc_rtc::log::info("[KinovaController] Constant velocity move complete.");
    currentTargetDistance_ = totalDistance_;
    currentTargetVelocity_ = 0.0;
    currentTargetAcceleration_ = 0.0;
    updateTasksRef();
    tf_ = 0.0;
    return;
  }

  currentTargetDistance_ = vMax_ * t_;
  currentTargetVelocity_ = (t_ < tf_) ? vMax_ : 0.0;
  currentTargetAcceleration_ = 0.0;

  updateTasksRef();
}

void KinovaController::initializeTrapezoidalVelocityMove(Eigen::Vector3d target, double distance_from_target, double v_max, double a_max)
{
  finalEEPosition_ = target - distance_from_target * basedDirectionVec;
  initialEEPosition_ = compEETask->positionTask->position();
  Eigen::Vector3d distance_from_EE = (finalEEPosition_ - initialEEPosition_);
  directionVec = (distance_from_EE).normalized();

  totalDistance_ = distance_from_EE.norm();

  vMax_ = v_max;
  aMax_ = a_max;

  t1_ = vMax_ / aMax_;
  d1_ = 0.5 * aMax_ * t1_ * t1_;

  if (2.0 * d1_ >= totalDistance_)
  {
    // TRIANGULAR: Dist too short to reach v_max
    mc_rtc::log::warning("[KinovaController] Trapezoidal velocity move cannot reach v_max (triangular profile). Set a lower v_max or a higher a_max.");
    tf_ = 0.0;
    return;
  }
  else
  {
    // TRAPEZOIDAL: Can reach v_max
    mc_rtc::log::info("[KinovaController] Trapezoidal velocity move initialized (trapezoidal profile).");
    d2_ = totalDistance_ - 2.0 * d1_;
    t2_ = d2_ / vMax_;
    tf_ = 2.0 * t1_ + t2_;
  }
  t_ = 0.0;
  mc_rtc::log::info("[KinovaController] Trapezoidal velocity move - Total time: {:.2f} s", tf_);
}

void KinovaController::trapezoidalVelocityMove()
{
  t_ += timeStep;

  if (t_ >= tf_)
  {
    mc_rtc::log::info("[KinovaController] Trapezoidal velocity move complete.");
    currentTargetDistance_ = totalDistance_;
    currentTargetVelocity_ = 0.0;
    currentTargetAcceleration_ = 0.0;
    updateTasksRef();
    tf_ = 0.0;
    return;
  }

  if (t_ <= t1_)
  {
    // Acceleration
    currentTargetDistance_     = 0.5 * aMax_ * t_ * t_;
    currentTargetVelocity_     = aMax_ * t_;
    currentTargetAcceleration_ = aMax_;
  }
  else if (t_ <= t1_ + t2_)
  {
    // Constant velocity
    double tau = t_ - t1_;
    currentTargetDistance_     = d1_ + vMax_ * tau;
    currentTargetVelocity_     = vMax_;
    currentTargetAcceleration_ = 0.0;
  }
  else
  {
    // Deceleration
    double tau = t_ - (t1_ + t2_);
    currentTargetDistance_ =
        d1_ + d2_ + vMax_ * tau - 0.5 * aMax_ * tau * tau;
    currentTargetVelocity_     = vMax_ - aMax_ * tau;
    currentTargetAcceleration_ = -aMax_;
  }

  updateTasksRef();
}

void KinovaController::initializeMinimumJerkMove(Eigen::Vector3d target,
                                                 double distance_from_target,
                                                 double v_max)
{
  finalEEPosition_   = target - distance_from_target * basedDirectionVec;
  initialEEPosition_ = compEETask->positionTask->position();

  Eigen::Vector3d distance_from_EE = finalEEPosition_ - initialEEPosition_;
  directionVec = distance_from_EE.normalized();

  totalDistance_ = distance_from_EE.norm();
  vMax_ = v_max;
  tf_ = 1.875 * totalDistance_ / vMax_;
  t_  = 0.0;

  mc_rtc::log::info("[KinovaController] Minimum jerk move initialized. Total time: {:.2f} s", tf_);
}

void KinovaController::minimumJerkMove()
{
  t_ += timeStep;

  if(t_ >= tf_) // Movement complete
  {
    mc_rtc::log::info("[KinovaController] Minimum jerk move complete.");

    currentTargetDistance_     = totalDistance_;
    currentTargetVelocity_     = 0.0;
    currentTargetAcceleration_ = 0.0;

    updateTasksRef();
    tf_ = 0.0;
    return;
  }

  double s = t_ / tf_;
  double s2 = s * s;
  double s3 = s2 * s;
  double s4 = s3 * s;
  double s5 = s4 * s;

  // Minimum jerk scalars
  currentTargetDistance_ =
      totalDistance_ * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);

  currentTargetVelocity_ =
      (totalDistance_ / tf_) * (30.0 * s2 - 60.0 * s3 + 30.0 * s4);

  currentTargetAcceleration_ =
      (totalDistance_ / (tf_ * tf_)) * (60.0 * s - 180.0 * s2 + 120.0 * s3);

  updateTasksRef();
}

void KinovaController::updateTasksRef()
{
  taskPosition = initialEEPosition_ + directionVec * currentTargetDistance_;
  taskVelocity = directionVec * currentTargetVelocity_;
  taskAcceleration = directionVec * currentTargetAcceleration_;

  compEETask->positionTask->position(taskPosition);
  compEETask->positionTask->refVel(taskVelocity);
  compEETask->positionTask->refAccel(taskAcceleration);
}

bool KinovaController::motionComplete() const
{
  return t_ >= tf_;
}