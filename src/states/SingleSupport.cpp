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

#include "SingleSupport.h"

namespace lipm_walking
{
  void states::SingleSupport::start()
  {
    auto & ctl = controller();
    auto & supportContact = ctl.supportContact();
    auto & targetContact = ctl.targetContact();

    earlyDoubleSupportDuration_ = 0.;
    duration_ = ctl.singleSupportDuration();
    hasUpdatedMPCOnce_ = false;
    remTime_ = ctl.singleSupportDuration();
    stateTime_ = 0.;
    timeSinceLastPreviewUpdate_ = 0.; // don't update at transition

    if (supportContact.surfaceName == "LeftFootCenter")
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
    swingFoot_.reset(
        swingFootTask->surfacePose(), targetContact.pose,
        duration_, ctl.plan.swingHeight());
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
    if (remTime_ < 0.)
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
    if (timeSinceLastPreviewUpdate_ > PREVIEW_UPDATE_PERIOD)
    {
      updatePreview();
    }

    ctl.preview->integrate(pendulum(), dt);
    if (hasUpdatedMPCOnce_)
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

    if (stabilizer().contactState() != ContactState::DoubleSupport)
    {
      bool liftPhase = (remTime_ > duration_ / 3.);
      bool touchdownDetected = stabilizer().detectTouchdown(swingFootTask, targetContact);
      if (liftPhase || !touchdownDetected)
      {
        swingFoot_.integrate(dt);
        swingFootTask->targetPose(swingFoot_.pose());
        swingFootTask->refVelB(swingFoot_.vel());
        swingFootTask->refAccel(swingFoot_.accel());
      }
      else // (stabilizer().contactState() != ContactState::DoubleSupport)
      {
        stabilizer().contactState(ContactState::DoubleSupport);
        stabilizer().setContact(swingFootTask, targetContact);
        earlyDoubleSupportDuration_ = remTime_;
      }
    }
  }

  void states::SingleSupport::updatePreview()
  {
    auto & ctl = controller();
    ctl.mpc().contacts(ctl.supportContact(), ctl.targetContact(), ctl.nextContact());
    if (ctl.isLastSSP() || ctl.pauseWalking)
    {
      ctl.nextDoubleSupportDuration(ctl.plan.finalDSPDuration());
      ctl.mpc().phaseDurations(remTime_, ctl.plan.finalDSPDuration(), 0.);
    }
    else
    {
      ctl.mpc().phaseDurations(remTime_, ctl.doubleSupportDuration(), ctl.singleSupportDuration());
    }
    if (ctl.updatePreview())
    {
      timeSinceLastPreviewUpdate_ = 0.;
      hasUpdatedMPCOnce_ = true;
    }
  }
}

EXPORT_SINGLE_STATE("SingleSupport", lipm_walking::states::SingleSupport)
