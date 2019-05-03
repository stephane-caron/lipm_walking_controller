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
  void states::Initial::start()
  {
    auto & ctl = controller();

    postureTaskIsActive_ = true;
    startStandingButton_ = false;
    startStanding_ = false;

    ctl.loadFootstepPlan(ctl.plan.name); // reload in case it was updated
    ctl.internalReset();

    logger().addLogEntry("walking_phase", []() { return -2.; });

    if (gui())
    {
      using namespace mc_rtc::gui;
      gui()->removeElement({"Walking", "Controller"}, "Pause walking");
      gui()->addElement(
        {"Walking", "Controller"},
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
    else
    {
      showStartStandingButton();
    }
  }

  bool states::Initial::checkTransitions()
  {
    if (startStanding_ && !postureTaskIsActive_)
    {
      output("Standing");
      return true;
    }
    return false;
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
