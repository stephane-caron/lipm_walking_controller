/*
 * Copyright (c) 2018-2019, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

    /** Get stabilizer.
     *
     */
    Stabilizer & stabilizer()
    {
      return controller_->stabilizer();
    }

    /** Start function.
     *
     */
    void start(mc_control::fsm::Controller & controller) override
    {
      controller_ = &static_cast<Controller&>(controller);
      start();
    }

    /** Teardown function.
     *
     */
    void teardown(mc_control::fsm::Controller &) override
    {
      teardown();
    }

    virtual bool checkTransitions() = 0;
    virtual void runState() = 0;
    virtual void start() = 0;
    virtual void teardown() = 0;

  protected:
    Controller * controller_ = nullptr;
  };
}
