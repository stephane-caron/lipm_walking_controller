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

#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/SwingFoot.h>
#include <lipm_walking/State.h>

namespace lipm_walking
{
  namespace states
  {
    /** Single support phase.
     *
     */
    struct SingleSupport : State
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

      /** Update swing foot target.
       *
       */
      void updateSwingFoot();

      /** Update horizontal MPC preview.
       *
       */
      void updatePreview();

    private:
      SwingFoot swingFoot_;
      bool hasUpdatedMPCOnce_;
      double duration_;
      double earlyDoubleSupportDuration_;
      double remTime_;
      double stateTime_;
      double timeSinceLastPreviewUpdate_;
      std::shared_ptr<mc_tasks::CoPTask> supportFootTask;
      std::shared_ptr<mc_tasks::CoPTask> swingFootTask;
    };
  }
}
