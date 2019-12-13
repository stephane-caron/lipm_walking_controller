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

#include "Initial.h"

namespace lipm_walking
{

void states::Initial::start()
{
  auto & ctl = controller();

  postureTaskIsActive_ = true;
  postureTaskWasActive_ = true;
  startStandingButton_ = false;
  startStanding_ = false;

  ctl.internalReset();

  logger().addLogEntry("walking_phase", []() { return -2.; });

  if(gui())
  {
    gui()->removeElement({"Walking", "Main"}, "Pause walking");
  }

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::Initial::teardown()
{
  logger().removeLogEntry("walking_phase");

  if(gui())
  {
    hideStartStandingButton();
  }
}

void states::Initial::runState()
{
  auto & ctl = controller();
  postureTaskIsActive_ = (ctl.postureTask->speed().norm() > 1e-2);
  if(postureTaskIsActive_)
  {
    hideStartStandingButton();
    postureTaskWasActive_ = true;
  }
  else if(postureTaskWasActive_)
  {
    ctl.internalReset();
    postureTaskWasActive_ = false;
  }
  else
  {
    showStartStandingButton();
  }
}

bool states::Initial::checkTransitions()
{
  if(startStanding_ && !postureTaskIsActive_)
  {
    output("Standing");
    return true;
  }
  return false;
}

void states::Initial::showStartStandingButton()
{
  if(!startStandingButton_ && gui())
  {
    using namespace mc_rtc::gui;
    gui()->addElement({"Walking", "Main"}, Button("Start standing", [this]() { startStanding_ = true; }));
    startStandingButton_ = true;
  }
}

void states::Initial::hideStartStandingButton()
{
  if(startStandingButton_ && gui())
  {
    gui()->removeElement({"Walking", "Main"}, "Start standing");
    startStandingButton_ = false;
  }
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("Initial", lipm_walking::states::Initial)
