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

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/SwingFoot.h>
#include <lipm_walking/State.h>

namespace lipm_walking
{
  namespace states
  {
    /** Single support phase while walking.
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
      SwingFoot swingFoot_; /**< Swing foot trajectory interpolator */
      bool hasUpdatedMPCOnce_; /**< Has the walking pattern been updated since the beginning of the SSP? */
      double duration_; /**< Total duration of the SSP in [s] */
      double remTime_; /**< Time remainin guntil the end of the phase */
      double stateTime_; /** Time since the beginning of the SSP */
      double timeSinceLastPreviewUpdate_; /**< Time count used to schedule MPC udpates, in [s] */
      std::shared_ptr<mc_tasks::force::CoPTask> supportFootTask; /**< Current support foot task from the stabilizer */
      std::shared_ptr<mc_tasks::force::CoPTask> swingFootTask; /**< Current swing foot task from the stabilizer */
    };
  }
}
