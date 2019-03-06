/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Stéphane Caron
 *
 * This file is part of lipm_walking_controller.
 *
 * lipm_walking_controller is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * lipm_walking_controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lipm_walking_controller. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "Initial.h"

namespace lipm_walking
{
  namespace
  {
    constexpr double MAX_ROBOT_MASS = 42.; // [kg]
    constexpr double MIN_ROBOT_MASS = 35.; // [kg]
  }

  void states::Initial::start()
  {
    auto & ctl = controller();

    isWeighing_ = true;
    massEstimator_.reset();
    pleaseReWeigh_ = false;
    postureTaskIsActive_ = true;
    startStandingButton_ = false;
    startStanding_ = false;

    ctl.loadFootstepPlan(ctl.plan.name); // reload in case it was updated
    ctl.internalReset();

    logger().addLogEntry("walking_phase", []() { return -2.; });

    if (gui())
    {
      using namespace mc_rtc::gui;
      gui()->removeElement({"Walking", "Controller"}, "Go back to standing");
      gui()->removeElement({"Walking", "Controller"}, "Pause walking");
      gui()->addElement(
        {"Walking", "Controller"},
        Button(
          "Weigh robot",
          [this]()
          {
            massEstimator_.reset();
            isWeighing_ = true;
            pleaseReWeigh_ = false;
          }),
        ComboInput("Footstep plan",
          ctl.availablePlans(),
          [&ctl]() { return ctl.plan.name; },
          [&ctl](const std::string & name)
          {
            ctl.loadFootstepPlan(name);
            ctl.internalReset();
          }));
    }

    runState(); // don't wait till next cycle to update reference and tasks
  }

  void states::Initial::teardown()
  {
    logger().removeLogEntry("walking_phase");

    if (gui())
    {
      gui()->removeElement({"Walking", "Controller"}, "Footstep plan");
      gui()->removeElement({"Walking", "Controller"}, "Weigh robot");
      hideStartStandingButton();
    }
  }

  void states::Initial::runState()
  {
    auto & ctl = controller();
    postureTaskIsActive_ = (ctl.postureTask->speed().norm() > 1e-2);
    if (postureTaskIsActive_)
    {
      ctl.internalReset();
      hideStartStandingButton();
    }
    else if (isWeighing_)
    {
      weighRobot();
      hideStartStandingButton();
    }
    else if (pleaseReWeigh_)
    {
      hideStartStandingButton();
    }
    else
    {
      showStartStandingButton();
    }
  }

  bool states::Initial::checkTransitions()
  {
    if (startStanding_ && !postureTaskIsActive_ && !isWeighing_)
    {
      output("Standing");
      return true;
    }
    return false;
  }

  void states::Initial::weighRobot()
  {
    auto & ctl = controller();
    double LFz = ctl.stabilizer().leftFootTask->measuredWrench().force().z();
    double RFz = ctl.stabilizer().rightFootTask->measuredWrench().force().z();
    massEstimator_.add((LFz + RFz) / world::GRAVITY);
    if (massEstimator_.n() > 100)
    {
      if (massEstimator_.avg() < MIN_ROBOT_MASS || massEstimator_.avg() > MAX_ROBOT_MASS)
      {
        LOG_ERROR("Negative mass: robot is in the air?");
        pleaseReWeigh_ = true;
      }
      else
      {
        ctl.updateRobotMass(massEstimator_.avg());
      }
      isWeighing_ = false;
    }
  }

  void states::Initial::showStartStandingButton()
  {
    if (!startStandingButton_ && gui())
    {
      using namespace mc_rtc::gui;
      gui()->addElement(
        {"Walking", "Controller"},
        Button("Start standing", [this]() { startStanding_ = true; }));
      startStandingButton_ = true;
    }
  }

  void states::Initial::hideStartStandingButton()
  {
    if (startStandingButton_ && gui())
    {
      gui()->removeElement({"Walking", "Controller"}, "Start standing");
      startStandingButton_ = false;
    }
  }
}

EXPORT_SINGLE_STATE("Initial", lipm_walking::states::Initial)
