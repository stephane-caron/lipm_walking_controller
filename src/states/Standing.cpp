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

#include <lipm_walking/utils/clamp.h>
#include <lipm_walking/utils/world.h>

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
    planChanged_ = false;
    lastInterpolatorIter_ = ctl.planInterpolator.nbIter;
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

    if (ctl.isLastDSP())
    {
      ctl.loadFootstepPlan(ctl.plan.name);
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
          ctl.planInterpolator.availablePlans(),
          [&ctl]() { return ctl.plan.name; },
          [this](const std::string & name) { updatePlan(name); }));
      gui()->addElement(
        {"Walking", "Controller"},
        Button(
          (supportContact.id == 0) ? "Start walking" : "Resume walking",
          [this]() { startWalking(); }));
      gui()->addElement(
        {"Standing"},
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
      gui()->removeCategory({"Standing"});
      gui()->removeElement({"Walking", "Controller"}, "Footstep plan");
      gui()->removeElement({"Walking", "Controller"}, "Gait");
      gui()->removeElement({"Walking", "Controller"}, "Go to middle");
      gui()->removeElement({"Walking", "Controller"}, "Resume walking");
      gui()->removeElement({"Walking", "Controller"}, "Start walking");
    }
  }

  void states::Standing::runState()
  {
    checkPlanUpdates();

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

    auto & ctl = controller();
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

  void states::Standing::checkPlanUpdates()
  {
    auto & ctl = controller();
    if (ctl.planInterpolator.nbIter > lastInterpolatorIter_)
    {
      ctl.loadFootstepPlan(ctl.planInterpolator.customPlanName());
      lastInterpolatorIter_ = ctl.planInterpolator.nbIter;
      planChanged_ = true;
    }
    if (planChanged_)
    {
      if (gui())
      {
        gui()->removeElement({"Walking", "Controller"}, "Resume walking");
        gui()->removeElement({"Walking", "Controller"}, "Start walking");
        gui()->addElement(
            {"Walking", "Controller"},
            mc_rtc::gui::Button("Start walking", [this]() { startWalking(); }));
      }
      planChanged_ = false;
    }
  }

  void states::Standing::updateTarget(double leftFootRatio)
  {
    auto & sole = controller().sole();
    if (controller().stabilizer().contactState() != ContactState::DoubleSupport)
    {
      LOG_ERROR("Cannot update CoM target while in single support");
      return;
    }
    leftFootRatio = clamp(leftFootRatio, 0., 1., "Standing target");
    sva::PTransformd X_0_lfr = sva::interpolate(rightFootContact_.anklePose(sole), leftFootContact_.anklePose(sole), leftFootRatio);
    copTarget_ = X_0_lfr.translation();
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

  void states::Standing::updatePlan(const std::string & name)
  {
    auto & ctl = controller();
    if (name.find("custom") != std::string::npos)
    {
      if (!ctl.customFootstepPlan)
      {
        ctl.planInterpolator.addGUIElements();
        ctl.customFootstepPlan = true;
      }
      if (name.find("backward") != std::string::npos)
      {
        ctl.planInterpolator.restoreBackwardTarget();
      }
      else if (name.find("forward") != std::string::npos)
      {
        ctl.planInterpolator.restoreForwardTarget();
      }
      else if (name.find("lateral") != std::string::npos)
      {
        ctl.planInterpolator.restoreLateralTarget();
      }
      ctl.loadFootstepPlan(ctl.planInterpolator.customPlanName());
    }
    else // new plan is not custom
    {
      if (ctl.customFootstepPlan)
      {
        ctl.planInterpolator.removeGUIElements();
        ctl.customFootstepPlan = false;
      }
      ctl.loadFootstepPlan(name);
    }
    planChanged_ = true;
  }
}

EXPORT_SINGLE_STATE("Standing", lipm_walking::states::Standing)
