/* Copyright 2018 CNRS-UM LIRMM
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

#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/State.h>
#include <lipm_walking/utils/AvgStdEstimator.h>

namespace lipm_walking
{
  /** Initial state.
   *
   * Check that contacts match foot positions.
   *
   */
  namespace states
  {
    struct Initial : State
    {
      /** Start state.
       *
       */
      void start() override;

      /** Teardown state.
       *
       */
      void teardown() override;

      /** Check transitions at beginning of control cycle.
       *
       */
      bool checkTransitions() override;

      /** Main state function, called if no transition at this cycle.
       *
       */
      void runState() override;

      /** Weigh robot based on force sensor readings. 
       *
       * Assumes the robot is standing still in double support on a flat
       * horizontal surface.
       */
      void weighRobot();

      /** Add "Start standing" transition button to GUI.
       *
       */
      void showStartStandingButton();

      /** Remove "Start standing" transition button from GUI.
       *
       */
      void hideStartStandingButton();

    private:
      AvgStdEstimator massEstimator_;
      bool isWeighing_;
      bool pleaseReWeigh_;
      bool postureTaskIsActive_;
      bool startStandingButton_;
      bool startStanding_;
    };
  }
}
