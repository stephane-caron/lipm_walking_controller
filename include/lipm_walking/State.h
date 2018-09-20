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

#include "Controller.h"

namespace lipm_walking
{
  /** Convenience wrapper for FSM states.
   *
   */
  struct State : mc_control::fsm::State
  {
    /** No configuration by default.
     *
     */
    void configure(const mc_rtc::Configuration &) override
    {
    }

    /** Start function.
     *
     */
    void start(mc_control::fsm::Controller & controller) override
    {
      controller_ = &static_cast<Controller&>(controller);
      start();
    }

    /** Main function.
     *
     */
    bool run(mc_control::fsm::Controller &) override
    {
      if (checkTransitions())
      {
        return true;
      }
      runState();
      return false;
    }

    /** Teardown function.
     *
     */
    void teardown(mc_control::fsm::Controller &) override
    {
      teardown();
    }

    /** Get controller.
     *
     */
    Controller & controller()
    {
      return *controller_;
    }

    /** Get GUI pointer.
     *
     */
    std::shared_ptr<mc_rtc::gui::StateBuilder> gui()
    {
      return controller_->gui();
    }

    /** Get logger.
     *
     */
    mc_rtc::Logger & logger()
    {
      return controller_->logger();
    }

    /** Get pendulum reference.
     *
     */
    Pendulum & pendulum()
    {
      return controller_->pendulum();
    }

    /** Get footstep plan.
     *
     */
    FootstepPlan & plan()
    {
      return controller_->plan;
    }

    /** Get stabilizer.
     *
     */
    Stabilizer & stabilizer()
    {
      return controller_->stabilizer();
    }

    virtual bool checkTransitions() = 0;
    virtual void runState() = 0;
    virtual void start() = 0;
    virtual void teardown() = 0;

  protected:
    Controller * controller_ = nullptr;
  };
}
