#include "KinovaController_EStop.h"

#include "../KinovaController.h"

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/logging.h>

#include <cmath>
#include <map>

static const std::vector<std::string> jointNames = {"joint_1", "joint_2", "joint_3", "joint_4",
                                                    "joint_5", "joint_6", "joint_7"};

void KinovaController_EStop::configure(const mc_rtc::Configuration & config)
{
  config("returnSpeed", returnSpeed_);
  config("minMoveTime", minMoveTime_);
  config("convergenceGrace", convergenceGrace_);
}

void KinovaController_EStop::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  if(!(returnSpeed_ > 0.0))
  {
    returnSpeed_ = 0.4;
  }
  // Stop: no EE task, posture target = the commanded configuration right now
  // (the commanded state is what the arm is tracking, so this is jump-free).
  ctl.solver().removeTask(ctl.compEETask);
  std::map<std::string, std::vector<double>> hold;
  for(const auto & jn : jointNames)
  {
    hold[jn] = {ctl.robot().mbc().q[ctl.robot().jointIndexByName(jn)][0]};
  }
  ctl.compPostureTask->target(hold);
  phase_ = HOLDING;
  returnRequested_ = false;
  mc_rtc::log::critical("[EStop] EMERGENCY STOP: arm stopped and holding. Gripper unchanged. "
                        "Press \"Emergency stop\" -> \"Return home (safe)\" to recover.");

  ctl.gui()->addElement({"KinovaController", "Emergency stop"},
                        mc_rtc::gui::Label("Status",
                                           [this]() {
                                             return phase_ == HOLDING ? std::string("STOPPED (holding)")
                                                                      : std::string("Returning home");
                                           }),
                        mc_rtc::gui::Button("Return home (safe)", [this]() { returnRequested_ = true; }));
}

bool KinovaController_EStop::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  switch(phase_)
  {
    case HOLDING:
    {
      if(!returnRequested_)
      {
        return false;
      }
      // Folding-corrected joint-space return home (same as Bottle1/NGPoint).
      constexpr double TWO_PI = 2.0 * mc_rtc::constants::PI;
      startQ_.clear();
      targetQ_.clear();
      double maxDelta = 0.0;
      for(const auto & jn : jointNames)
      {
        const double ref = ctl.robot().mbc().q[ctl.robot().jointIndexByName(jn)][0];
        double target = ctl.postureHome.at(jn)[0];
        target += TWO_PI * std::round((ref - target) / TWO_PI);
        startQ_.push_back(ref);
        targetQ_.push_back(target);
        maxDelta = std::max(maxDelta, std::abs(target - ref));
      }
      tf_ = std::max(1.875 * maxDelta / returnSpeed_, minMoveTime_);
      t_ = 0.0;
      phase_ = RETURNING;
      mc_rtc::log::info("[EStop] Returning home safely ({:.1f} s)", tf_);
      return false;
    }
    case RETURNING:
    {
      t_ += ctl.timeStep;
      const double u = std::min(t_ / tf_, 1.0);
      const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
      std::map<std::string, std::vector<double>> target;
      double maxErr = 0.0;
      for(size_t i = 0; i < jointNames.size(); ++i)
      {
        target[jointNames[i]] = {startQ_[i] + s * (targetQ_[i] - startQ_[i])};
        const double q = ctl.robot().mbc().q[ctl.robot().jointIndexByName(jointNames[i])][0];
        maxErr = std::max(maxErr, std::abs(q - targetQ_[i]));
      }
      ctl.compPostureTask->target(target);
      if(t_ >= tf_ && (maxErr < 0.05 || t_ > tf_ + convergenceGrace_))
      {
        mc_rtc::log::info("[EStop] Back at home.");
        phase_ = DONE;
      }
      return false;
    }
    case DONE:
      output("OK");
      return true;
  }
  return false;
}

void KinovaController_EStop::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<KinovaController &>(ctl_);
  ctl.gui()->removeElement({"KinovaController", "Emergency stop"}, "Status");
  ctl.gui()->removeElement({"KinovaController", "Emergency stop"}, "Return home (safe)");
}

EXPORT_SINGLE_STATE("KinovaController_EStop", KinovaController_EStop)
