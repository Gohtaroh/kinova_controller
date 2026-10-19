#include "KinovaController.h"
#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/logging.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>

namespace
{
double steadyNow()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace

KinovaController::KinovaController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  frame_ = config("frame", (std::string) "end_effector_link");

  // Handover-condition parameters (speed scale + trajectory blend alpha per
  // condition). Overridable from the controller config so the pilot can retune
  // the levels without recompiling; sizes must stay consistent with the names.
  config("handoverSpeedScales", handoverSpeedScales_);
  config("handoverAlphas", handoverAlphas_);
  config("handoverArmDelays", handoverArmDelays_);
  config("handoverDurationFactor", handoverDurationFactor_);
  // > 0 or every move duration becomes zero/negative (FSM would stall or jump).
  if(!(handoverDurationFactor_ > 0.0))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[KinovaController] handoverDurationFactor = {} must be > 0",
                                                     handoverDurationFactor_);
  }
  mc_rtc::log::info("[KinovaController] Move duration factor {:.4f} for all conditions (1.4375 = Baseline timing)",
                    handoverDurationFactor_);
  if(handoverSpeedScales_.size() != handoverConditionNames_.size()
     || handoverAlphas_.size() != handoverConditionNames_.size()
     || handoverArmDelays_.size() != handoverConditionNames_.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[KinovaController] handoverSpeedScales / handoverAlphas / handoverArmDelays must each have {} entries",
        handoverConditionNames_.size());
  }
  // Value ranges: alpha must stay in [0,1] (outside it the blended time scaling
  // reverses direction and 'speed' stops meaning peak velocity); speed scale must
  // be strictly positive (a zero scale makes the move duration infinite and stalls
  // the FSM, a negative one is silently floored to a wrong duration). Fail fast so a
  // bad config can never feed a reversed or NaN posture target to the arm.
  for(size_t i = 0; i < handoverConditionNames_.size(); ++i)
  {
    if(!(handoverAlphas_[i] >= 0.0 && handoverAlphas_[i] <= 1.0))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[KinovaController] handoverAlphas[{}] = {} out of range [0, 1]",
                                                       i, handoverAlphas_[i]);
    }
    if(!(handoverSpeedScales_[i] > 0.0))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[KinovaController] handoverSpeedScales[{}] = {} must be > 0", i,
                                                       handoverSpeedScales_[i]);
    }
    if(!(handoverArmDelays_[i] >= 0.0))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[KinovaController] handoverArmDelays[{}] = {} must be >= 0", i,
                                                       handoverArmDelays_[i]);
    }
  }

  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.1, 9.0});
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  postureHome = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                 {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};

  // Create the compliant posture task
  compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 100.0, 1.0);
  solver().addTask(compPostureTask);

  // Create the compliant end-effector task
  compEETask =
      std::make_shared<mc_tasks::CompliantEndEffectorTask>(frame_, robots(), robot().robotIndex(), 100.0, 1000.0);

  compEETask->reset();
  solver().addTask(compEETask);

  homeOrientation = Eigen::Quaterniond(1, -1, -1, -1).normalized().toRotationMatrix(); // Based orientation
  taskOrientation = Eigen::Quaterniond(-0.271, 0.653, 0.653, 0.271).normalized().toRotationMatrix();

  homePosition = compEETask->positionTask->position();
  taskPosition = homePosition;
  taskVelocity = Eigen::Vector3d::Zero();
  taskAcceleration = Eigen::Vector3d::Zero();

  // Define EE target positions
  cardPosition_1 = Eigen::Vector3d(0.4, 0.3, 0.2); // robot-side (closer) + to the side (2026-07-23)
  cardPosition_2 = Eigen::Vector3d(0.7, -0.2, 0.2);
  cardPosition_3 = Eigen::Vector3d(0.7, 0.2, 0.2);

  // Disable the mc_rtc gripper divergence safety on this reversed gripper: on trigger
  // the release offset is applied in the wrong direction, causing chatter during the
  // close and snapping the gripper open (observed on both MuJoCo and the real robot,
  // 2026-07-06; confirmed working with the threshold maxed out from the GUI).
  // Force protection is handled elsewhere anyway: the real Robotiq 2F-85 limits its
  // grasp force internally, and the MuJoCo actuator is force-capped in the model.
  if(robot().hasGripper("gripper"))
  {
    auto & gripper = robot().gripper("gripper");
    gripper.actualCommandDiffTrigger(10.0); // joint range is 0.8 rad, can never trigger
    if(gripper.overCommandLimitIterN() <= 1)
    {
      // Real-robot module defaults command at 99% of the URDF joint velocity; keep the
      // close reasonably quick but not slamming (real Robotiq limits grasp force internally)
      gripper.percentVMAX(0.5);
    }
  }

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
  gui()->addElement({"KinovaController", "Movement"},
                    mc_rtc::gui::ArrayInput(
                        "Direction Vector", {"x", "y", "z"}, [this]() { return directionVec; },
                        [this](const Eigen::Vector3d & dir) { directionVec = dir; }));

  // Handover motion-style selector: pick the condition before triggering a pick.
  // Lives at the TOP of the "Bottle picks" panel (added before the button row, and
  // elements render in insertion order) so the operator drives the whole session —
  // condition, NG, and the four bottles — from one panel. Moved out of its own
  // "Handover" category 2026-07-23 for the pilot.
  gui()->addElement({"KinovaController", "Bottle picks"},
                    mc_rtc::gui::ComboInput(
                        "Condition", handoverConditionNames_,
                        [this]() -> const std::string & { return handoverConditionNames_[handoverCondition_]; },
                        [this](const std::string & name)
                        {
                          for(size_t i = 0; i < handoverConditionNames_.size(); ++i)
                          {
                            if(handoverConditionNames_[i] == name)
                            {
                              handoverCondition_ = i;
                            }
                          }
                          mc_rtc::log::info("[KinovaController] Handover condition -> {} (speed x{:.3f}, alpha {:.2f})",
                                            handoverConditionName(), handoverSpeedScale(), handoverAlpha());
                        }));

  gui()->addElement({"KinovaController", "Movement"}, mc_rtc::gui::Point3DRO("Final Position", finalEEPosition_));
  gui()->addElement({"KinovaController", "Card Positions"}, mc_rtc::gui::Point3DRO("Card Position 1", cardPosition_1));
  gui()->addElement({"KinovaController", "Card Positions"}, mc_rtc::gui::Point3DRO("Card Position 2", cardPosition_2));
  gui()->addElement({"KinovaController", "Card Positions"}, mc_rtc::gui::Point3DRO("Card Position 3", cardPosition_3));

  // Ordered one-button pick panel, laid out like the physical bench: NG on its own
  // row, then Bottle1..4 left-to-right below. The FSM's own "Force transition to X"
  // buttons come out in std::unordered_set hash order (mc_rtc Executor::next), so
  // Bottle1..4 show up jumbled (e.g. 4,2,3,1); these fixed-order buttons replace
  // them for daily use. Each triggers the SAME transition (executor_.resume, what
  // a force-transition button calls) and is guarded to Initial so it can never
  // interrupt a running pick. Labels are display-only and overridable from the
  // controller config (recipe contents change per session); the resume target is
  // the real state name regardless of the label.
  // NOTE (mc_rtc GUI limits): buttons are plain text buttons — there is no square
  // or size control and no precise inter-button spacing; all five sit on one
  // horizontal row in physical order: NG, then Bottle1..4 left-to-right.
  // Default labels follow README recipe A (Japanese-style): slots 1/2 are fixed
  // across all recipes (salad oil / vinegar), slots 3/4 change per recipe (A: soy
  // sauce / granulated sugar; B: mayonnaise / sesame; E: ketchup / sugar), so set
  // bottleButtonLabels per session for the active recipe.
  std::vector<std::string> bottleLabels =
      config("bottleButtonLabels", std::vector<std::string>{"1（サラダ油）", "2（酢）", "3（醤油マヨケチャップ）",
                                                            "4（グラニュー糖白すりごま）"});
  if(bottleLabels.size() != 4)
  {
    mc_rtc::log::warning("[KinovaController] bottleButtonLabels must have 4 entries (got {}), using defaults",
                         bottleLabels.size());
    bottleLabels = {"1（サラダ油）", "2（酢）", "3（醤油マヨケチャップ）", "4（グラニュー糖白すりごま）"};
  }
  std::string ngLabel = config("ngButtonLabel", (std::string) "NG 指差し");
  auto pickButton = [this](const std::string & label, const std::string & target)
  {
    return mc_rtc::gui::Button(label,
                               [this, target]()
                               {
                                 if(executor_.state() == "KinovaController_Initial")
                                 {
                                   executor_.resume(target);
                                 }
                                 else
                                 {
                                   mc_rtc::log::warning(
                                       "[KinovaController] '{}' ignored: pick buttons only work from Initial "
                                       "(current: {})",
                                       target, executor_.state());
                                 }
                               });
  };
  // One horizontal row: NG, then the four bottles left-to-right in physical order.
  gui()->addElement({"KinovaController", "Bottle picks"}, mc_rtc::gui::ElementsStacking::Horizontal,
                    pickButton(ngLabel, "KinovaController_NGPoint"), pickButton(bottleLabels[0], "Bottle1"),
                    pickButton(bottleLabels[1], "Bottle2"), pickButton(bottleLabels[2], "Bottle3"),
                    pickButton(bottleLabels[3], "Bottle4"));

  // Software emergency stop: loopback UDP listener fed by scripts/estop_mouse.py
  // (mouse middle-click). Non-blocking, polled once per control tick in run().
  config("estopPort", estopPort_);
  estopSock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if(estopSock_ >= 0)
  {
    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(estopPort_));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if(::bind(estopSock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0
       || ::fcntl(estopSock_, F_SETFL, ::fcntl(estopSock_, F_GETFL, 0) | O_NONBLOCK) != 0)
    {
      mc_rtc::log::error("[KinovaController] Emergency-stop listener could not bind 127.0.0.1:{} ({}); "
                         "mouse emergency stop DISABLED",
                         estopPort_, std::strerror(errno));
      ::close(estopSock_);
      estopSock_ = -1;
    }
    else
    {
      mc_rtc::log::info("[KinovaController] Emergency-stop listener on 127.0.0.1:{}", estopPort_);
    }
  }
  // Start the mouse watcher with the controller so the operator never has to.
  // A watcher started by hand as well is harmless (both just send ESTOP/ALIVE).
  if(estopSock_ >= 0 && config("estopWatcherAutostart", true))
  {
    const char * home = std::getenv("HOME");
    const std::string defaultScript =
        std::string(home ? home : "") + "/workspace/src/kinova_controller/scripts/estop_mouse.py";
    startEstopWatcher(config("estopWatcherScript", defaultScript));
  }
  gui()->addElement({"KinovaController", "Emergency stop"},
                    mc_rtc::gui::Label("Mouse middle-click stop",
                                       [this]()
                                       {
                                         if(estopSock_ < 0)
                                         {
                                           return std::string("DISABLED (listener failed)");
                                         }
                                         return estopWatcherAlive()
                                                    ? std::string("ACTIVE")
                                                    : std::string(
                                                        "NOT RUNNING (watcher failed to start: see terminal)");
                                       }));

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
  logger().addLogEntry("KinovaController_PlannerDistance",
                       [this]() { return (finalEEPosition_ - cardPosition_1).norm(); });
  // Log the active handover condition so the pilot can report as-implemented
  // kinematics (speed scale, blend alpha) per condition for the manipulation check.
  logger().addLogEntry("KinovaController_HandoverAlpha", [this]() { return handoverAlpha(); });
  logger().addLogEntry("KinovaController_HandoverSpeedScale", [this]() { return handoverSpeedScale(); });
  logger().addLogEntry("KinovaController_HandoverDurationFactor", [this]() { return handoverDurationFactor(); });

  mc_rtc::log::success("KinovaController init done ");
}

KinovaController::~KinovaController()
{
  if(estopWatcherPid_ > 0)
  {
    ::kill(estopWatcherPid_, SIGTERM);
    for(int i = 0; i < 20 && ::waitpid(estopWatcherPid_, nullptr, WNOHANG) == 0; ++i)
    {
      ::usleep(50000);
    }
  }
  if(estopSock_ >= 0)
  {
    ::close(estopSock_);
  }
}

void KinovaController::startEstopWatcher(const std::string & script)
{
  if(!std::getenv("DISPLAY"))
  {
    mc_rtc::log::warning("[KinovaController] No DISPLAY: mouse emergency-stop watcher not started");
    return;
  }
  if(::access(script.c_str(), R_OK) != 0)
  {
    mc_rtc::log::error("[KinovaController] Mouse emergency-stop watcher not found: {}", script);
    return;
  }
  const std::string port = std::to_string(estopPort_);
  const pid_t pid = ::fork();
  if(pid == 0)
  {
    // Child: die with the controller process, then become the watcher.
    ::prctl(PR_SET_PDEATHSIG, SIGTERM);
    ::execlp("python3", "python3", script.c_str(), "--port", port.c_str(), static_cast<char *>(nullptr));
    ::_exit(127);
  }
  if(pid < 0)
  {
    mc_rtc::log::error("[KinovaController] Could not start the mouse emergency-stop watcher ({})",
                       std::strerror(errno));
    return;
  }
  estopWatcherPid_ = pid;
  mc_rtc::log::info("[KinovaController] Mouse emergency-stop watcher started (pid {}): {}", pid, script);
}

bool KinovaController::estopWatcherAlive() const
{
  return steadyNow() - estopLastAlive_ < 2.0;
}

void KinovaController::pollEmergencyStop()
{
  if(estopSock_ < 0)
  {
    return;
  }
  char buf[32];
  for(int n = 0; n < 64; ++n) // drain everything queued this tick
  {
    const ssize_t len = ::recv(estopSock_, buf, sizeof(buf) - 1, 0);
    if(len <= 0)
    {
      break;
    }
    buf[len] = '\0';
    if(std::strncmp(buf, "ALIVE", 5) == 0)
    {
      if(!estopWatcherAlive())
      {
        mc_rtc::log::success("[KinovaController] Mouse emergency-stop watcher connected");
      }
      estopLastAlive_ = steadyNow();
    }
    else if(std::strncmp(buf, "ESTOP", 5) == 0)
    {
      estopLastAlive_ = steadyNow();
      // The switch happens on the next FSM tick, so ignore repeats until then.
      if(executor_.state() != "KinovaController_EStop" && !estopPending_)
      {
        estopPending_ = true;
        mc_rtc::log::critical("[KinovaController] EMERGENCY STOP (mouse middle-click) during {}", executor_.state());
        executor_.resume("KinovaController_EStop");
      }
    }
  }
}

bool KinovaController::run()
{
  if(estopPending_ && executor_.state() == "KinovaController_EStop")
  {
    estopPending_ = false;
  }
  pollEmergencyStop();
  distanceEECard1_ = (compEETask->positionTask->position() - cardPosition_1).norm();
  distanceEECard2_ = (compEETask->positionTask->position() - cardPosition_2).norm();
  distanceEECard3_ = (compEETask->positionTask->position() - cardPosition_3).norm();

  // Update the solver depending on the control mode
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if(ctrl_mode.compare("Position") == 0)
  {
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
  t_ = 0.0;
  mc_rtc::log::info("[KinovaController] Constant velocity move initialized. Total time: {:.2f} s", tf_);
}

void KinovaController::constantVelocityMove()
{
  t_ += timeStep;

  if(t_ >= tf_)
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

void KinovaController::initializeTrapezoidalVelocityMove(Eigen::Vector3d target,
                                                         double distance_from_target,
                                                         double v_max,
                                                         double a_max)
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

  if(2.0 * d1_ >= totalDistance_)
  {
    // TRIANGULAR: Dist too short to reach v_max
    mc_rtc::log::warning("[KinovaController] Trapezoidal velocity move cannot reach v_max (triangular profile). Set a "
                         "lower v_max or a higher a_max.");
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

  if(t_ >= tf_)
  {
    mc_rtc::log::info("[KinovaController] Trapezoidal velocity move complete.");
    currentTargetDistance_ = totalDistance_;
    currentTargetVelocity_ = 0.0;
    currentTargetAcceleration_ = 0.0;
    updateTasksRef();
    tf_ = 0.0;
    return;
  }

  if(t_ <= t1_)
  {
    // Acceleration
    currentTargetDistance_ = 0.5 * aMax_ * t_ * t_;
    currentTargetVelocity_ = aMax_ * t_;
    currentTargetAcceleration_ = aMax_;
  }
  else if(t_ <= t1_ + t2_)
  {
    // Constant velocity
    double tau = t_ - t1_;
    currentTargetDistance_ = d1_ + vMax_ * tau;
    currentTargetVelocity_ = vMax_;
    currentTargetAcceleration_ = 0.0;
  }
  else
  {
    // Deceleration
    double tau = t_ - (t1_ + t2_);
    currentTargetDistance_ = d1_ + d2_ + vMax_ * tau - 0.5 * aMax_ * tau * tau;
    currentTargetVelocity_ = vMax_ - aMax_ * tau;
    currentTargetAcceleration_ = -aMax_;
  }

  updateTasksRef();
}

void KinovaController::initializeMinimumJerkMove(Eigen::Vector3d target, double distance_from_target, double v_max)
{
  finalEEPosition_ = target - distance_from_target * basedDirectionVec;
  initialEEPosition_ = compEETask->positionTask->position();

  Eigen::Vector3d distance_from_EE = finalEEPosition_ - initialEEPosition_;
  directionVec = distance_from_EE.normalized();

  totalDistance_ = distance_from_EE.norm();
  vMax_ = v_max;
  tf_ = 1.875 * totalDistance_ / vMax_;
  t_ = 0.0;

  mc_rtc::log::info("[KinovaController] Minimum jerk move initialized. Total time: {:.2f} s", tf_);
}

void KinovaController::minimumJerkMove()
{
  t_ += timeStep;

  if(t_ >= tf_) // Movement complete
  {
    mc_rtc::log::info("[KinovaController] Minimum jerk move complete.");

    currentTargetDistance_ = totalDistance_;
    currentTargetVelocity_ = 0.0;
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
  currentTargetDistance_ = totalDistance_ * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);

  currentTargetVelocity_ = (totalDistance_ / tf_) * (30.0 * s2 - 60.0 * s3 + 30.0 * s4);

  currentTargetAcceleration_ = (totalDistance_ / (tf_ * tf_)) * (60.0 * s - 180.0 * s2 + 120.0 * s3);

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
