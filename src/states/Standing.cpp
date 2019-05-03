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

#include <lipm_walking/utils/clamp.h>

#include "Standing.h"

namespace lipm_walking
{
  namespace
  {
    constexpr double COM_STIFFNESS = 5.; // standing has CoM set-point task
  }

  void states::Standing::start()
  {
    auto & ctl = controller();
    auto & supportContact = ctl.supportContact();
    auto & targetContact = ctl.targetContact();

    freeFootGain_ = 30.;
    isMakingFootContact_ = false;
    leftFootRatio_ = ctl.leftFootRatio();
    releaseHeight_ = 0.05; // [m]
    startWalking_ = false;
    if (supportContact.surfaceName == "RightFootCenter")
    {
      leftFootContact_ = targetContact;
      rightFootContact_ = supportContact;
    }
    else if (supportContact.surfaceName == "LeftFootCenter")
    {
      leftFootContact_ = supportContact;
      rightFootContact_ = targetContact;
    }
    else
    {
      LOG_ERROR_AND_THROW(std::invalid_argument,
          "Unknown surface name: " << supportContact.surfaceName);
    }

    stabilizer().contactState(ContactState::DoubleSupport);
    stabilizer().setContact(stabilizer().leftFootTask, leftFootContact_);
    stabilizer().setContact(stabilizer().rightFootTask, rightFootContact_);
    stabilizer().addTasks(ctl.solver());
    ctl.solver().addTask(ctl.pelvisTask);
    ctl.solver().addTask(ctl.torsoTask);

    updateTarget(leftFootRatio_);

    logger().addLogEntry("support_xmax", [&ctl]() { return std::max(ctl.supportContact().xmax(), ctl.targetContact().xmax()); });
    logger().addLogEntry("support_xmin", [&ctl]() { return std::min(ctl.supportContact().xmin(), ctl.targetContact().xmin()); });
    logger().addLogEntry("support_ymax", [&ctl]() { return std::max(ctl.supportContact().ymax(), ctl.targetContact().ymax()); });
    logger().addLogEntry("support_ymin", [&ctl]() { return std::min(ctl.supportContact().ymin(), ctl.targetContact().ymin()); });
    logger().addLogEntry("support_zmax", [&ctl]() { return std::max(ctl.supportContact().zmax(), ctl.targetContact().zmax()); });
    logger().addLogEntry("support_zmin", [&ctl]() { return std::min(ctl.supportContact().zmin(), ctl.targetContact().zmin()); });
    logger().addLogEntry("walking_phase", []() { return 3.; });
    ctl.stopLogSegment();

    if (gui())
    {
      using namespace mc_rtc::gui;
      gui()->removeElement({"Walking", "Controller"}, "Pause walking");
      gui()->addElement(
        {"Walking", "Controller"},
        ComboInput("Footstep plan",
          ctl.availablePlans(),
          [&ctl]() { return ctl.plan.name; },
          [&ctl](const std::string & name)
          {
            ctl.loadFootstepPlan(name);
          }));
      gui()->addElement(
        {"Walking", "Controller"},
        Button(
          (supportContact.id == 0 || ctl.isLastDSP()) ? "Start walking" : "Resume walking",
          [this]() { startWalking(); }));
      gui()->addElement(
        {"Walking", "Standing"},
        NumberInput(
          "CoM target [0-1]",
          [this]() { return std::round(leftFootRatio_ * 10.) / 10.; },
          [this](double leftFootRatio) { updateTarget(leftFootRatio); }),
        NumberInput(
          "Free foot gain",
          [this]() { return std::round(freeFootGain_); },
          [this](double gain) { freeFootGain_ = clamp(gain, 5., 100.); }),
        NumberInput(
          "Release height [m]",
          [this]() { return std::round(releaseHeight_ * 100.) / 100.; },
          [this](double height) { releaseHeight_ = clamp(height, 0., 0.25); }),
        Label(
          "Left foot pressure [N]",
          [&ctl]() { return ctl.realRobot().forceSensor("LeftFootForceSensor").force().z(); }),
        Label(
          "Right foot pressure [N]",
          [&ctl]() { return ctl.realRobot().forceSensor("RightFootForceSensor").force().z(); }),
        Button(
          "Go to left foot",
          [this]() { updateTarget(1.); }),
        Button(
          "Go to middle",
          [this]() { updateTarget(0.5); }),
        Button(
          "Go to right foot",
          [this]() { updateTarget(0.); }),
        Button(
          "Make left foot contact",
          [this]() { makeLeftFootContact(); }),
        Button(
          "Make right foot contact",
          [this]() { makeRightFootContact(); }),
        Button(
          "Release left foot",
          [this]() { releaseLeftFootContact(); }),
        Button(
          "Release right foot",
          [this]() { releaseRightFootContact(); })
      );
    }

    runState(); // don't wait till next cycle to update reference and tasks
  }

  void states::Standing::teardown()
  {
    auto & ctl = controller();

    stabilizer().removeTasks(ctl.solver());

    logger().removeLogEntry("support_xmax");
    logger().removeLogEntry("support_xmin");
    logger().removeLogEntry("support_ymax");
    logger().removeLogEntry("support_ymin");
    logger().removeLogEntry("support_zmax");
    logger().removeLogEntry("support_zmin");
    logger().removeLogEntry("walking_phase");

    if (gui())
    {
      gui()->removeCategory({"Walking", "Standing"});
      gui()->removeElement({"Walking", "Controller"}, "Footstep plan");
      gui()->removeElement({"Walking", "Controller"}, "Go to middle");
      gui()->removeElement({"Walking", "Controller"}, "Resume walking");
      gui()->removeElement({"Walking", "Controller"}, "Start walking");
    }
  }

  void states::Standing::runState()
  {
    auto & ctl = controller();

    if (isMakingFootContact_)
    {
      auto & leftFootTask = stabilizer().leftFootTask;
      auto & rightFootTask = stabilizer().rightFootTask;
      bool isLeftFootSeekingContact = (leftFootTask->admittance().couple().x() < 1e-10);
      bool isRightFootSeekingContact = (rightFootTask->admittance().couple().x() < 1e-10);
      bool leftFootTouchdown = stabilizer().detectTouchdown(leftFootTask, leftFootContact_);
      bool rightFootTouchdown = stabilizer().detectTouchdown(rightFootTask, rightFootContact_);
      if (isLeftFootSeekingContact && leftFootTouchdown)
      {
        stabilizer().setContact(leftFootTask, leftFootContact_);
      }
      if (isRightFootSeekingContact && rightFootTouchdown)
      {
        stabilizer().setContact(rightFootTask, rightFootContact_);
      }
      isLeftFootSeekingContact = (leftFootTask->admittance().couple().x() < 1e-10);
      isRightFootSeekingContact = (rightFootTask->admittance().couple().x() < 1e-10);
      if (!isLeftFootSeekingContact && !isRightFootSeekingContact)
      {
        isMakingFootContact_ = false;
        stabilizer().contactState(ContactState::DoubleSupport);
      }
    }

    auto & pendulum = ctl.pendulum();

    Eigen::Vector3d comTarget = copTarget_ + Eigen::Vector3d{0., 0., ctl.plan.comHeight()};
    const Eigen::Vector3d & com_i = pendulum.com();
    const Eigen::Vector3d & comd_i = pendulum.comd();
    const Eigen::Vector3d & cop_f = copTarget_;

    double K = COM_STIFFNESS;
    double D = 2 * std::sqrt(K);
    Eigen::Vector3d comdd = K * (comTarget - com_i) - D * comd_i;
    Eigen::Vector3d n = ctl.supportContact().normal();
    double lambda = n.dot(comdd - world::gravity) / n.dot(com_i - cop_f);
    Eigen::Vector3d zmp = com_i + (world::gravity - comdd) / lambda;

    pendulum.integrateIPM(zmp, lambda, ctl.timeStep);
    ctl.leftFootRatio(leftFootRatio_);
    ctl.stabilizer().run();
  }

  void states::Standing::updateTarget(double leftFootRatio)
  {
    if (controller().stabilizer().contactState() != ContactState::DoubleSupport)
    {
      LOG_ERROR("Cannot update CoM target while in single support");
      return;
    }
    leftFootRatio = clamp(leftFootRatio, 0., 1., "Standing target");
    sva::PTransformd X_0_mid = sva::interpolate(rightFootContact_.anklePose(), leftFootContact_.anklePose(), leftFootRatio);
    copTarget_ = X_0_mid.translation();
    leftFootRatio_ = leftFootRatio;
  }

  void states::Standing::makeFootContact(std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact)
  {
    auto & stabilizer = controller().stabilizer();
    if (footTask->admittance().couple().x() > 1e-10 || stabilizer.detectTouchdown(footTask, contact))
    {
      LOG_WARNING("Foot is already in contact");
      return;
    }
    stabilizer.setSwingFoot(footTask);
    stabilizer.seekTouchdown(footTask);
    footTask->stiffness(freeFootGain_); // sets damping as well
    footTask->targetPose(contact.pose);
    isMakingFootContact_ = true;
  }

  void states::Standing::makeLeftFootContact()
  {
    makeFootContact(controller().stabilizer().leftFootTask, leftFootContact_);
  }

  void states::Standing::makeRightFootContact()
  {
    makeFootContact(controller().stabilizer().rightFootTask, rightFootContact_);
  }

  bool states::Standing::releaseFootContact(std::shared_ptr<mc_tasks::force::CoPTask> footTask)
  {
    constexpr double MAX_FOOT_RELEASE_PRESSURE = 50.; // [N]
    auto & stabilizer = controller().stabilizer();
    if (footTask->admittance().couple().x() < 1e-10)
    {
      LOG_WARNING("Foot contact is already released");
      return false;
    }
    else if (footTask->measuredWrench().force().z() > MAX_FOOT_RELEASE_PRESSURE)
    {
      LOG_ERROR("Contact pressure is too high to release foot");
      return false;
    }
    sva::PTransformd X_0_f = footTask->surfacePose();
    sva::PTransformd X_f_t = Eigen::Vector3d{0., 0., releaseHeight_};
    stabilizer.setSwingFoot(footTask);
    footTask->stiffness(freeFootGain_); // sets damping as well
    footTask->targetPose(X_f_t * X_0_f);
    return true;
  }

  void states::Standing::releaseLeftFootContact()
  {
    if (releaseFootContact(controller().stabilizer().leftFootTask))
    {
      controller().stabilizer().contactState(ContactState::RightFoot);
    }
  }

  void states::Standing::releaseRightFootContact()
  {
    if (releaseFootContact(controller().stabilizer().rightFootTask))
    {
      controller().stabilizer().contactState(ContactState::LeftFoot);
    }
  }

  bool states::Standing::checkTransitions()
  {
    auto & ctl = controller();

    if (isMakingFootContact_)
    {
      return false;
    }
    if (!startWalking_)
    {
      return false;
    }

    ctl.mpc().contacts(ctl.supportContact(), ctl.targetContact(), ctl.nextContact());
    ctl.mpc().phaseDurations(0., ctl.plan.initDSPDuration(), ctl.singleSupportDuration());
    if (ctl.updatePreview())
    {
      ctl.nextDoubleSupportDuration(ctl.plan.initDSPDuration());
      ctl.startLogSegment(ctl.plan.name);
      output("DoubleSupport");
      return true;
    }
    return false;
  }

  void states::Standing::startWalking()
  {
    auto & ctl = controller();
    if (ctl.isLastSSP())
    {
      LOG_ERROR("No footstep in contact plan");
      return;
    }
    startWalking_ = true;
    gui()->addElement(
      {"Walking", "Controller"},
      mc_rtc::gui::Button(
        "Pause walking",
        [&ctl]() { ctl.pauseWalkingCallback(/* verbose = */ true); }));
  }
}

EXPORT_SINGLE_STATE("Standing", lipm_walking::states::Standing)
