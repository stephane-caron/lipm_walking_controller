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

#include "SingleSupport.h"

namespace lipm_walking
{

void states::SingleSupport::start()
{
  auto & ctl = controller();
  auto & supportContact = ctl.supportContact();
  auto & targetContact = ctl.targetContact();

  duration_ = ctl.singleSupportDuration();
  hasUpdatedMPCOnce_ = false;
  remTime_ = ctl.singleSupportDuration();
  stateTime_ = 0.;
  timeSinceLastPreviewUpdate_ = 0.; // don't update at transition

  if(supportContact.surfaceName == "LeftFootCenter")
  {
    ctl.leftFootRatio(1.);
    stabilizer().contactState(ContactState::LeftFoot);
    supportFootTask = stabilizer().leftFootTask;
    swingFootTask = stabilizer().rightFootTask;
  }
  else // (ctl.supportContact.surfaceName == "RightFootCenter")
  {
    ctl.leftFootRatio(0.);
    stabilizer().contactState(ContactState::RightFoot);
    supportFootTask = stabilizer().rightFootTask;
    swingFootTask = stabilizer().leftFootTask;
  }

  swingFoot_.landingDuration(ctl.plan.landingDuration());
  swingFoot_.landingPitch(ctl.plan.landingPitch());
  swingFoot_.takeoffDuration(ctl.plan.takeoffDuration());
  swingFoot_.takeoffOffset(ctl.plan.takeoffOffset());
  swingFoot_.takeoffPitch(ctl.plan.takeoffPitch());
  swingFoot_.reset(swingFootTask->surfacePose(), targetContact.pose, duration_, ctl.plan.swingHeight());
  stabilizer().setContact(supportFootTask, supportContact);
  stabilizer().setSwingFoot(swingFootTask);
  stabilizer().addTasks(ctl.solver());

  logger().addLogEntry("rem_phase_time", [this]() { return remTime_; });
  logger().addLogEntry("support_xmax", [&ctl]() { return ctl.supportContact().xmax(); });
  logger().addLogEntry("support_xmin", [&ctl]() { return ctl.supportContact().xmin(); });
  logger().addLogEntry("support_ymax", [&ctl]() { return ctl.supportContact().ymax(); });
  logger().addLogEntry("support_ymin", [&ctl]() { return ctl.supportContact().ymin(); });
  logger().addLogEntry("support_zmax", [&ctl]() { return ctl.supportContact().zmax(); });
  logger().addLogEntry("support_zmin", [&ctl]() { return ctl.supportContact().zmin(); });
  logger().addLogEntry("walking_phase", []() { return 1.; });
  swingFoot_.addLogEntries(logger());

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::SingleSupport::teardown()
{
  stabilizer().removeTasks(controller().solver());

  logger().removeLogEntry("contact_impulse");
  logger().removeLogEntry("rem_phase_time");
  logger().removeLogEntry("support_xmax");
  logger().removeLogEntry("support_xmin");
  logger().removeLogEntry("support_ymax");
  logger().removeLogEntry("support_ymin");
  logger().removeLogEntry("support_zmax");
  logger().removeLogEntry("support_zmin");
  logger().removeLogEntry("walking_phase");
  swingFoot_.removeLogEntries(logger());
}

bool states::SingleSupport::checkTransitions()
{
  if(remTime_ < 0.)
  {
    output("DoubleSupport");
    return true;
  }
  return false;
}

void states::SingleSupport::runState()
{
  auto & ctl = controller();
  double dt = ctl.timeStep;

  updateSwingFoot();
  if(timeSinceLastPreviewUpdate_ > PREVIEW_UPDATE_PERIOD)
  {
    updatePreview();
  }

  ctl.preview->integrate(pendulum(), dt);
  if(hasUpdatedMPCOnce_)
  {
    pendulum().resetCoMHeight(ctl.plan.comHeight(), ctl.supportContact());
    pendulum().completeIPM(ctl.supportContact());
  }
  else // still in DSP of preview
  {
    pendulum().completeIPM(ctl.prevContact());
  }
  stabilizer().run();

  remTime_ -= dt;
  stateTime_ += dt;
  timeSinceLastPreviewUpdate_ += dt;
}

void states::SingleSupport::updateSwingFoot()
{
  auto & ctl = controller();
  auto & targetContact = ctl.targetContact();
  double dt = ctl.timeStep;

  if(stabilizer().contactState() != ContactState::DoubleSupport)
  {
    bool liftPhase = (remTime_ > duration_ / 3.);
    bool touchdownDetected = stabilizer().detectTouchdown(swingFootTask, targetContact);
    if(liftPhase || !touchdownDetected)
    {
      swingFoot_.integrate(dt);
      swingFootTask->targetPose(swingFoot_.pose());
      // The linear velocity in swingFoot_.vel() is expressed in the world
      // frame, but refVelB expects a body velocity, so the following lines are
      // wrong (they miss a frame transform). Thanks to @mmurooka for pointing
      // out this bug <https://github.com/jrl-umi3218/lipm_walking_controller/issues/29>.
      swingFootTask->refVelB(swingFoot_.vel());
      swingFootTask->refAccel(swingFoot_.accel());
    }
    else // (stabilizer().contactState() != ContactState::DoubleSupport)
    {
      stabilizer().contactState(ContactState::DoubleSupport);
      stabilizer().setContact(swingFootTask, targetContact);
    }
  }
}

void states::SingleSupport::updatePreview()
{
  auto & ctl = controller();
  ctl.mpc().contacts(ctl.supportContact(), ctl.targetContact(), ctl.nextContact());
  if(ctl.isLastSSP() || ctl.pauseWalking)
  {
    ctl.nextDoubleSupportDuration(ctl.plan.finalDSPDuration());
    ctl.mpc().phaseDurations(remTime_, ctl.plan.finalDSPDuration(), 0.);
  }
  else
  {
    ctl.mpc().phaseDurations(remTime_, ctl.doubleSupportDuration(), ctl.singleSupportDuration());
  }
  if(ctl.updatePreview())
  {
    timeSinceLastPreviewUpdate_ = 0.;
    hasUpdatedMPCOnce_ = true;
  }
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("SingleSupport", lipm_walking::states::SingleSupport)
