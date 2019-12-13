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

#include "DoubleSupport.h"

namespace lipm_walking
{

void states::DoubleSupport::start()
{
  auto & ctl = controller();

  double phaseDuration = ctl.doubleSupportDuration(); // careful! side effect here

  duration_ = phaseDuration;
  initLeftFootRatio_ = ctl.leftFootRatio();
  remTime_ = (phaseDuration > ctl.timeStep) ? phaseDuration : -ctl.timeStep;
  stateTime_ = 0.;
  stopDuringThisDSP_ = ctl.pauseWalking;
  if(phaseDuration > ctl.timeStep)
  {
    timeSinceLastPreviewUpdate_ = 2 * PREVIEW_UPDATE_PERIOD; // update at transition...
  }
  else // ... unless DSP duration is zero
  {
    timeSinceLastPreviewUpdate_ = 0.;
  }

  const std::string & targetSurfaceName = ctl.targetContact().surfaceName;
  auto actualTargetPose = ctl.controlRobot().surfacePose(targetSurfaceName);
  ctl.plan.goToNextFootstep(actualTargetPose);
  if(ctl.isLastDSP()) // called after goToNextFootstep
  {
    stopDuringThisDSP_ = true;
  }

  stabilizer().contactState(ContactState::DoubleSupport);
  if(ctl.prevContact().surfaceName == "LeftFootCenter")
  {
    stabilizer().setContact(stabilizer().leftFootTask, ctl.prevContact());
    stabilizer().setContact(stabilizer().rightFootTask, ctl.supportContact());
    targetLeftFootRatio_ = 0.;
  }
  else // (ctl.prevContact().surfaceName == "RightFootCenter")
  {
    stabilizer().setContact(stabilizer().leftFootTask, ctl.supportContact());
    stabilizer().setContact(stabilizer().rightFootTask, ctl.prevContact());
    targetLeftFootRatio_ = 1.;
  }
  if(stopDuringThisDSP_)
  {
    targetLeftFootRatio_ = 0.5;
  }
  stabilizer().addTasks(ctl.solver());

  logger().addLogEntry("rem_phase_time", [this]() { return remTime_; });
  logger().addLogEntry("support_xmax",
                       [&ctl]() { return std::max(ctl.prevContact().xmax(), ctl.supportContact().xmax()); });
  logger().addLogEntry("support_xmin",
                       [&ctl]() { return std::min(ctl.prevContact().xmin(), ctl.supportContact().xmin()); });
  logger().addLogEntry("support_ymax",
                       [&ctl]() { return std::max(ctl.prevContact().ymax(), ctl.supportContact().ymax()); });
  logger().addLogEntry("support_ymin",
                       [&ctl]() { return std::min(ctl.prevContact().ymin(), ctl.supportContact().ymin()); });
  logger().addLogEntry("support_zmax",
                       [&ctl]() { return std::max(ctl.prevContact().zmax(), ctl.supportContact().zmax()); });
  logger().addLogEntry("support_zmin",
                       [&ctl]() { return std::min(ctl.prevContact().zmin(), ctl.supportContact().zmin()); });
  logger().addLogEntry("walking_phase", []() { return 2.; });

  if(stopDuringThisDSP_)
  {
    ctl.pauseWalking = false;
  }

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::DoubleSupport::teardown()
{
  stabilizer().removeTasks(controller().solver());

  logger().removeLogEntry("rem_phase_time");
  logger().removeLogEntry("support_xmax");
  logger().removeLogEntry("support_xmin");
  logger().removeLogEntry("support_ymax");
  logger().removeLogEntry("support_ymin");
  logger().removeLogEntry("support_zmax");
  logger().removeLogEntry("support_zmin");
  logger().removeLogEntry("walking_phase");
}

void states::DoubleSupport::runState()
{
  auto & ctl = controller();
  double dt = ctl.timeStep;

  if(remTime_ > 0 && timeSinceLastPreviewUpdate_ > PREVIEW_UPDATE_PERIOD
     && !(stopDuringThisDSP_ && remTime_ < PREVIEW_UPDATE_PERIOD))
  {
    updatePreview();
  }

  double x = clamp(remTime_ / duration_, 0., 1.);
  ctl.leftFootRatio(x * initLeftFootRatio_ + (1. - x) * targetLeftFootRatio_);

  ctl.preview->integrate(pendulum(), dt);
  pendulum().completeIPM(ctl.prevContact());
  pendulum().resetCoMHeight(ctl.plan.comHeight(), ctl.prevContact());
  stabilizer().run();

  remTime_ -= dt;
  stateTime_ += dt;
  timeSinceLastPreviewUpdate_ += dt;
}

bool states::DoubleSupport::checkTransitions()
{
  auto & ctl = controller();
  if(!stopDuringThisDSP_ && remTime_ < 0.)
  {
    output("SingleSupport");
    return true;
  }
  if(stopDuringThisDSP_ && remTime_ < -0.5)
  {
    if(!ctl.isLastDSP())
    {
      ctl.plan.restorePreviousFootstep(); // current one is for next SSP
    }
    output("Standing");
    return true;
  }
  return false;
}

void states::DoubleSupport::updatePreview()
{
  auto & ctl = controller();
  ctl.mpc().contacts(ctl.prevContact(), ctl.supportContact(), ctl.targetContact());
  if(stopDuringThisDSP_)
  {
    ctl.mpc().phaseDurations(0., remTime_, 0.);
  }
  else
  {
    ctl.mpc().phaseDurations(0., remTime_, ctl.singleSupportDuration());
  }
  if(ctl.updatePreview())
  {
    timeSinceLastPreviewUpdate_ = 0.;
  }
  else
  {
    LOG_WARNING("No capture trajectory, resuming walking");
    stopDuringThisDSP_ = false;
  }
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("DoubleSupport", lipm_walking::states::DoubleSupport)
