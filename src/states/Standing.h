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
#include <lipm_walking/HorizontalMPCProblem.h>
#include <lipm_walking/State.h>

namespace lipm_walking
{
  namespace states
  {
    /** Standing phase.
     *
     * Applies a simple CoM set-point task:
     *
     *    comdd = stiffness * (comTarget - com) - damping * comd
     *
     * with critical damping = 2 * sqrt(stiffness).
     *
     */
    struct Standing : State
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

      /** Distribute spatial ZMP into foot CoPs in double support.
       *
       */
      void distributeFootCoPs();

      /** Update target CoM and CoP.
       *
       * \param leftFootRatio Left foot weight index between 0 and 1.
       *
       */
      void updateTarget(double leftFootRatio);

      /** Make foot contact.
       *
       * \param footTask Stabilizer task corresponding to the foot to release.
       *
       * \param contact Contact target.
       *
       */
      void makeFootContact(std::shared_ptr<mc_tasks::CoPTask> footTask, const Contact & contact);

      /** Make left foot contact.
       *
       */
      void makeLeftFootContact();

      /** Make right foot contact.
       *
       */
      void makeRightFootContact();

      /** Release foot contact.
       *
       * \param footTask Stabilizer task corresponding to the foot to release.
       *
       * \return True if the contact was released, false if not.
       */
      bool releaseFootContact(std::shared_ptr<mc_tasks::CoPTask> footTask);

      /** Release left foot contact.
       *
       */
      void releaseLeftFootContact();

      /** Release right foot contact.
       *
       */
      void releaseRightFootContact();

      /** Enable startWalking_ boolean and update GUI.
       *
       */
      void startWalking();

    private:
      Contact leftFootContact_;
      Contact rightFootContact_;
      Eigen::Vector3d copTarget_;
      bool goToZeroStep_;
      bool isMakingFootContact_;
      bool startWalking_;
      double freeFootGain_;
      double leftFootRatio_;
      double releaseHeight_;
      unsigned nbDistribFail_;
    };
  }
}
