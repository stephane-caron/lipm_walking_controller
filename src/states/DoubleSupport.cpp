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

#include "DoubleSupport.h"

namespace lipm_walking
{
  void states::DoubleSupport::start()
  {
    auto & ctl = controller();

    duration_ = ctl.doubleSupportDuration();
    initLeftFootRatio_ = ctl.leftFootRatio();
    remTime_ = duration_;
    stateTime_ = 0.;
    stopDuringThisDSP_ = ctl.pauseWalking || ctl.prevContact().pauseAfterSwing;
    timeSinceLastPreviewUpdate_ = 2 * HorizontalMPC::SAMPLING_PERIOD; // update at transition

    const std::string & targetSurfaceName = ctl.targetContact().surfaceName;
    auto actualTargetPose = ctl.controlRobot().surfacePose(targetSurfaceName);
    ctl.plan.goToNextFootstep(actualTargetPose);
    if (ctl.isLastDSP()) // called after goToNextFootstep
    {
      stopDuringThisDSP_ = true;
    }

    stabilizer().contactState(ContactState::DoubleSupport);
    if (ctl.prevContact().surfaceName == "LeftFootCenter")
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
    if (stopDuringThisDSP_)
    {
      targetLeftFootRatio_ = 0.5;
    }
    stabilizer().addTasks(ctl.solver());

    logger().addLogEntry("rem_phase_time", [this]() { return remTime_; });
    logger().addLogEntry("support_xmax", [&ctl]() { return std::max(ctl.prevContact().xmax(), ctl.supportContact().xmax()); });
    logger().addLogEntry("support_xmin", [&ctl]() { return std::min(ctl.prevContact().xmin(), ctl.supportContact().xmin()); });
    logger().addLogEntry("support_ymax", [&ctl]() { return std::max(ctl.prevContact().ymax(), ctl.supportContact().ymax()); });
    logger().addLogEntry("support_ymin", [&ctl]() { return std::min(ctl.prevContact().ymin(), ctl.supportContact().ymin()); });
    logger().addLogEntry("support_zmax", [&ctl]() { return std::max(ctl.prevContact().zmax(), ctl.supportContact().zmax()); });
    logger().addLogEntry("support_zmin", [&ctl]() { return std::min(ctl.prevContact().zmin(), ctl.supportContact().zmin()); });
    logger().addLogEntry("walking_phase", []() { return 2.; });

    if (stopDuringThisDSP_)
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

    if (remTime_ > 0 && timeSinceLastPreviewUpdate_ > HorizontalMPC::SAMPLING_PERIOD &&
        !(stopDuringThisDSP_ && remTime_ < 1 * HorizontalMPC::SAMPLING_PERIOD))
    {
      updatePreview();
    }

    double x = clamp(remTime_ / duration_, 0., 1.);
    ctl.leftFootRatio(x * initLeftFootRatio_ + (1. - x) * targetLeftFootRatio_);

    ctl.preview->integrate(pendulum(), dt);
    pendulum().completeIPM(ctl.prevContact());
    double height = ctl.comHeight();
    pendulum().resetCoMHeight(height, ctl.prevContact());
    stabilizer().run();

    remTime_ -= dt;
    stateTime_ += dt;
    timeSinceLastPreviewUpdate_ += dt;
  }

  bool states::DoubleSupport::checkTransitions()
  {
    auto & ctl = controller();
    if (!stopDuringThisDSP_ && remTime_ < 0.)
    {
      output("SingleSupport");
      return true;
    }
    if (stopDuringThisDSP_ && remTime_ < -0.5)
    {
      if (!ctl.isLastDSP())
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
    ctl.hmpc.contacts(ctl.prevContact(), ctl.supportContact(), ctl.targetContact());
    if (stopDuringThisDSP_)
    {
      ctl.hmpc.phaseDurations(0., remTime_, 0.);
    }
    else
    {
      ctl.hmpc.phaseDurations(0., remTime_, ctl.singleSupportDuration());
    }
    if (ctl.updatePreview())
    {
      timeSinceLastPreviewUpdate_ = 0.;
    }
  }
}

EXPORT_SINGLE_STATE("DoubleSupport", lipm_walking::states::DoubleSupport)
