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

#include <lipm_walking/Contact.h>
#include <lipm_walking/PlanInterpolator.h>
#include <lipm_walking/utils/clamp.h>

namespace lipm_walking
{
  inline double floorn(double x, int n)
  {
    return floor(pow(10, n) * x) / pow(10, n);
  }

  void PlanInterpolator::addGUIElements()
  {
    using namespace mc_rtc::gui;
    // Ensure that a footstep marker independent from the map is used everytime
    // a plan finishes
    targetMap_ = false;
    gui_->addElement(
      {"Walking", "Planning"},
      NumberInput(
        "Initial angle [deg]",
        [this]() { return -initPose_.theta * 180. / M_PI; },
        [this](double angle)
        {
          initPose_.theta = clamp(-angle * M_PI / 180., -2 * M_PI, 2 * M_PI);
          run();
        }),
      ArrayInput(
        "Local target", {"x [m]", "y [m]", "theta [deg]"},
        [this]()
        {
          return targetPose_.vectorDegrees();
        },
        [this](const Eigen::Vector3d & desired)
        {
          updateLocalTarget_(SE2d(desired.x(), desired.y(), desired.z() * M_PI / 180.));
          run();
        }),
      ComboInput(
        "Gait",
        {"Sagittal", "Lateral", "InPlace"},
        [this]() { return gait(); },
        [this](const std::string & dir) { gait(dir); }),
      ComboInput(
        "Lead foot",
        {"Left", "Right"},
        [this]()
        {
          return (startWithRightFootstep_) ? "Right" : "Left";
        },
        [this](const std::string & footName)
        {
          startWithRightFootstep_ = (footName == "Right");
          run();
        }),
      NumberInput(
        "Desired step angle [deg]",
        [this]() { return desiredStepAngle_ * 180. / M_PI; },
        [this](double angleDeg)
        {
          desiredStepAngle_ = clamp(angleDeg, 0., IN_PLACE_MAX_STEP_ANGLE) * M_PI / 180.;
          run();
        }),
      NumberInput(
        "Desired step length [m]",
        [this]() { return desiredStepLength_; },
        [this](double length)
        {
          bool isLateral = (customPlan_.name == "custom_lateral");
          double maxLength = (isLateral) ? MAX_LATERAL_STEP_LENGTH : MAX_SAGITTAL_STEP_LENGTH;
          desiredStepLength_ = clamp(length, MIN_STEP_LENGTH, maxLength);
          run();
        }),
      NumberInput(
        "Extra step width [m]",
        [this]() { return extraStepWidth_; },
        [this](double width)
        {
          extraStepWidth_ = clamp(width, 0., MAX_EXTRA_STEP_WIDTH);
          run();
        }),
      NumberInput(
        "Scale init. vel.",
        [this]() { return supportPath_.extraInitVelScaling(); },
        [this](double s)
        {
          supportPath_.extraInitVelScaling(s);
          run();
        }),
      NumberInput(
        "Scale target vel.",
        [this]() { return supportPath_.extraTargetVelScaling(); },
        [this](double s)
        {
          supportPath_.extraTargetVelScaling(s);
          run();
        }),
      Label(
        "Number of steps",
        [this]() { return nbFootsteps_; }),
      Label(
        "Step angle [deg]",
        [this]() { return std::round(stepAngle_ * 180. / M_PI * 10.) / 10.; }),
      Label(
        "Step length [m]",
        [this]() { return std::round(stepLength_ * 1000.) / 1000.; }),
      Label(
        "Total length [m]",
        [this]() { return std::round(supportPath_.arcLength(0., 1.) * 1000.) / 1000.; }));
    gui_->addElement(
      {"Walking", "Advanced", "Markers", "Planning"},
      Trajectory(
        "Support_Path",
        [this]() -> const std::vector<Eigen::Vector3d> & { return supportPathDisplay_; }));
    gui_->addElement(
      {"Walking", "Planning"},
      XYTheta(
        "World target [m, rad]",
        [this]() -> Eigen::VectorXd
        {
          Eigen::Vector3d targetLocal;
          targetLocal << targetPose_.pos(), 0.;
          Eigen::Matrix3d rotLocal = mc_rbdyn::rpyToMat({0.,0.,targetPose_.theta});
          sva::PTransformd targetWorld = sva::PTransformd(rotLocal, targetLocal) * worldReference_;
          double thetaWorld = mc_rbdyn::rpyFromMat(targetWorld.rotation()).z();
          Eigen::VectorXd vec(4);
          vec << floorn(targetWorld.translation().x(), 4), floorn(targetWorld.translation().y(), 4), floorn(thetaWorld,4), worldReference_.translation().z();
          return vec;
        },
        [this](const Eigen::VectorXd & desired)
        {
          updateWorldTarget_(desired.head<3>());
        }));
  }

  void PlanInterpolator::updateWorldTarget_(const Eigen::Vector3d& desired)
  {
    Eigen::Vector3d posWorld;
    posWorld << desired(0), desired(1), worldReference_.translation().z();
    Eigen::Matrix3d rotWorld = mc_rbdyn::rpyToMat({0.,0.,desired(2)});
    sva::PTransformd X_desired(rotWorld, posWorld);

    sva::PTransformd targetLocal = X_desired * worldReference_.inv();
    double thetaLocal = mc_rbdyn::rpyFromMat(targetLocal.rotation()).z();
    const Eigen::Vector3d& posLocal = targetLocal.translation();
    updateLocalTarget_(SE2d(posLocal.x(), posLocal.y(), thetaLocal));
    run();
  }

  void PlanInterpolator::updateLocalTarget_(const SE2d & target)
  {
    targetPose_.x = floorn(clamp(target.pos().x(), -5., 5.), 4);
    targetPose_.y = floorn(clamp(target.pos().y(), -5., 5.), 4);
    targetPose_.theta = floorn(clamp(target.theta, -M_PI, M_PI), 4);
    suggestGait();
  }

  void PlanInterpolator::suggestGait()
  {
    double absX = std::abs(targetPose_.x);
    double absY = std::abs(targetPose_.y);
    if (absX > 2. * absY)
    {
      gait_ = Gait::Sagittal;
    }
    else // (absY >= 0.5 * absX)
    {
      gait_ = Gait::Lateral;
    }
  }

  void PlanInterpolator::configure(const mc_rtc::Configuration & plans)
  {
    plans_ = plans;
    for (auto name : {"custom_backward", "custom_forward", "custom_lateral"})
    {
      if (!plans_.has(name))
      {
        LOG_ERROR("[PlanInterpolator] Configuration lacks \"" << name << "\" plan, skipping...");
        continue;
      }
      customPlan_ = plans_(name);
      const Contact & leftFoot = customPlan_.contacts()[1];
      const Contact & rightFoot = customPlan_.contacts()[0];
      if (leftFoot.surfaceName != "LeftFootCenter" || rightFoot.surfaceName != "RightFootCenter")
      {
        LOG_ERROR("Wrong initial foothold order in \"" << name << "\" plan");
      }
      sva::PTransformd X_0_mid = sva::interpolate(leftFoot.pose, rightFoot.pose, 0.5);
      if (!X_0_mid.rotation().isIdentity() || X_0_mid.translation().norm() > 1e-4)
      {
        LOG_ERROR("Invalid X_0_mid(\"" << name << "\") = " << X_0_mid);
      }
    }

    // start with custom forward plan
    desiredStepLength_ = plans_("custom_forward")("step_length");
    customPlan_ = plans_("custom_forward");
    customPlan_.name = "custom_forward";
    run();
  }

  void PlanInterpolator::run()
  {
    if (targetPose_.pos().norm() < 2e-3)
    {
      if (gait_ != Gait::InPlace)
      {
        extraStepWidth_ = IN_PLACE_EXTRA_STEP_WIDTH;
        gait_ = Gait::InPlace;
      }
    }
    else if (gait_ == Gait::InPlace)
    {
      extraStepWidth_ = DEFAULT_EXTRA_STEP_WIDTH;
      gait_ = Gait::Lateral;
    }
    if (gait_ == Gait::Sagittal)
    {
      double dx = targetPose_.x;
      if (dx < 0. && customPlan_.name != "custom_backward")
      {
        desiredStepLength_ = plans_("custom_backward")("step_length");
        restoreDefaults();
      }
      else if (dx >= 0. && customPlan_.name != "custom_forward")
      {
        desiredStepLength_ = plans_("custom_forward")("step_length");
        restoreDefaults();
      }
      runSagittal_();
    }
    else if (gait_ == Gait::Lateral)
    {
      if (customPlan_.name != "custom_lateral")
      {
        desiredStepLength_ = plans_("custom_lateral")("step_length");
        restoreDefaults();
      }
      runLateral_();
    }
    else // (gait_ == Gait::InPlace)
    {
      if (customPlan_.name != "custom_lateral")
      {
        desiredStepLength_ = plans_("custom_lateral")("step_length");
        extraStepWidth_ = IN_PLACE_EXTRA_STEP_WIDTH;
      }
      runInPlace_();
    }
    nbIter++;
  }

  void PlanInterpolator::runSagittal_()
  {
    bool goingBackward = (targetPose_.pos().x() < 0.);
    if (goingBackward)
    {
      customPlan_ = plans_("custom_backward");
      customPlan_.name = "custom_backward";
      supportPath_.reset(initPose_.pos(), -initPose_.ori(), targetPose_.pos(), -targetPose_.ori());
      lastBackwardTarget_ = targetPose_;
    }
    else // going forward
    {
      customPlan_ = plans_("custom_forward");
      customPlan_.name = "custom_forward";
      supportPath_.reset(initPose_.pos(), initPose_.ori(), targetPose_.pos(), targetPose_.ori());
      lastForwardTarget_ = targetPose_;
    }
    if (!startWithRightFootstep_)
    {
      customPlan_.swapFirstTwoContacts();
    }

    double T = customPlan_.doubleSupportDuration() + customPlan_.singleSupportDuration();
    double totalLength = supportPath_.arcLength(0., 1.);
    unsigned nbInnerSteps = static_cast<unsigned>(std::max(0., std::round(totalLength / desiredStepLength_) - 1));
    double maxStepLength = 1.1 * desiredStepLength_;
    if (totalLength / (nbInnerSteps + 1) > maxStepLength)
    {
      nbInnerSteps++;
    }
    double innerStepLength = totalLength / (nbInnerSteps + 1);
    double innerVel = (goingBackward ? -1. : +1.) * innerStepLength / T;
    double outerStepLength;
    double outerVel;
    if (nbInnerSteps > 0)
    {
      outerStepLength = 0.5 * innerStepLength;
      outerVel = 0.5 * innerVel;
      stepAngle_ = 0.;
      stepLength_ = innerStepLength;
    }
    else
    {
      outerStepLength = totalLength;
      outerVel = innerVel;
      stepAngle_ = 0.;
      stepLength_ = totalLength;
    }

    nbFootsteps_ = 0;
    bool isRightFootstep = startWithRightFootstep_;
    unsigned nbFinalSteps = 0;
    double freeLength = outerStepLength;
    double curVel = outerVel;
    while (nbFinalSteps < 2)
    {
      double length = freeLength;
      double curStepWidth = stepWidth_ + extraStepWidth_;
      if (length > totalLength - outerStepLength - 1e-3)
      {
        curVel = outerVel;
      }
      if (length >= totalLength - 1e-3)
      {
        curStepWidth = stepWidth_;
        curVel = 0.;
        length = totalLength;
        nbFinalSteps++;
      }

      double t = supportPath_.arcLengthInverse(0., length);
      Eigen::Vector2d supportPoint = supportPath_.pos(t);
      Eigen::Vector2d supportTangent = supportPath_.tangent(t);
      if (goingBackward)
      {
        supportTangent *= -1.;
      }
      Eigen::Vector2d supportNormal = {-supportTangent.y(), supportTangent.x()};
      double sign = (isRightFootstep) ? -1. : +1.;
      double dy = 0.5 * sign * curStepWidth;
      double theta = atan2(supportTangent.y(), supportTangent.x());
      Eigen::Vector2d p = supportPoint + dy * supportNormal;
      SE2d stepPose = {p.x(), p.y(), theta};

      Contact contact;
      contact.pose = stepPose.asPTransform();
      contact.refVel = curVel * Eigen::Vector3d{supportTangent.x(), supportTangent.y(), 0.};
      contact.surfaceName = (isRightFootstep) ? "RightFootCenter" : "LeftFootCenter";
      customPlan_.appendContact(contact);
      isRightFootstep = !isRightFootstep;
      nbFootsteps_++;
      freeLength += innerStepLength;
      curVel = innerVel;
    }

    if (nbInnerSteps > 0)
    {
      if (nbFootsteps_ - 3 != nbInnerSteps)
      {
        LOG_ERROR("[PlanInterpolator] Footstep count check failed");
      }
      if (std::abs(2 * outerStepLength + nbInnerSteps * innerStepLength - totalLength) > 1e-4)
      {
        LOG_ERROR("[PlanInterpolator] Total length check failed");
      }
    }
    else // (nbInnerSteps == 0)
    {
      if (nbFootsteps_ != 2)
      {
        LOG_ERROR("[PlanInterpolator] Footstep count check failed");
      }
      if (std::abs(outerStepLength - totalLength) > 1e-4)
      {
        LOG_ERROR("[PlanInterpolator] Total length check failed");
      }
    }
  }

  void PlanInterpolator::runLateral_()
  {
    customPlan_ = plans_("custom_lateral");
    customPlan_.name = "custom_lateral";
    lastLateralTarget_ = targetPose_;
    supportPath_.reset(initPose_.pos(), initPose_.ori(), targetPose_.pos(), targetPose_.ori());

    std::string leadFootSurfaceName, followFootSurfaceName;
    bool goingToTheRight = (targetPose_.pos().y() < 0.);
    if (goingToTheRight)
    {
      leadFootSurfaceName = "RightFootCenter";
      followFootSurfaceName = "LeftFootCenter";
      startWithRightFootstep_ = true;
    }
    else // first footstep is left foot
    {
      leadFootSurfaceName = "LeftFootCenter";
      followFootSurfaceName = "RightFootCenter";
      customPlan_.swapFirstTwoContacts();
      startWithRightFootstep_ = false;
    }

    double totalLength = supportPath_.arcLength(0., 1.);
    double nbSteps = std::max(1., std::round(totalLength / desiredStepLength_));
    double maxStepLength = 1.1 * desiredStepLength_;
    if (totalLength / nbSteps > maxStepLength)
    {
      nbSteps++;
    }
    double stepLength = totalLength / nbSteps;

    nbFootsteps_ = 0;
    for (unsigned i = 1; i <= nbSteps; i++)
    {
      double curStepWidth = stepWidth_;
      if (i < nbSteps)
      {
        curStepWidth += extraStepWidth_;
      }
      double t = supportPath_.arcLengthInverse(0., i * stepLength);
      Eigen::Vector2d supportPoint = supportPath_.pos(t);
      double theta = t * targetPose_.theta;
      Eigen::Vector2d lateralVec = {-std::sin(theta), std::cos(theta)};
      lateralVec *= (goingToTheRight ? -1. : +1.) * 0.5 * curStepWidth;
      Eigen::Vector2d leadPos = supportPoint + lateralVec;
      Eigen::Vector2d followPos = supportPoint - lateralVec;
      SE2d leadStepPose = {leadPos.x(), leadPos.y(), theta};
      SE2d followStepPose = {followPos.x(), followPos.y(), theta};

      Contact followStep, leadStep;
      followStep.pose = followStepPose.asPTransform();
      followStep.surfaceName = followFootSurfaceName;
      leadStep.pose = leadStepPose.asPTransform();
      leadStep.surfaceName = leadFootSurfaceName;
      customPlan_.appendContact(leadStep);
      customPlan_.appendContact(followStep);
      nbFootsteps_ += 2;
    }

    stepAngle_ = 0.;
    stepLength_ = stepLength;
  }

  void PlanInterpolator::runInPlace_()
  {
    customPlan_ = plans_("custom_lateral");
    customPlan_.name = "custom_lateral";
    supportPath_.reset(initPose_.pos(), initPose_.ori(), targetPose_.pos(), targetPose_.ori());

    std::string leadFootSurfaceName, followFootSurfaceName;
    bool goingToTheRight = (targetPose_.theta < 0.);
    if (goingToTheRight)
    {
      leadFootSurfaceName = "RightFootCenter";
      followFootSurfaceName = "LeftFootCenter";
      startWithRightFootstep_ = true;
    }
    else // first footstep is left foot
    {
      leadFootSurfaceName = "LeftFootCenter";
      followFootSurfaceName = "RightFootCenter";
      customPlan_.swapFirstTwoContacts();
      startWithRightFootstep_ = false;
    }

    double totalAngle = std::abs(targetPose_.theta);
    double nbSteps = std::max(1., std::round(totalAngle / desiredStepAngle_));
    double maxStepAngle = 1.0 * desiredStepAngle_;
    if (totalAngle / nbSteps > maxStepAngle)
    {
      nbSteps++;
    }
    double stepAngle = totalAngle / nbSteps;

    nbFootsteps_ = 0;
    for (unsigned i = 1; i <= nbSteps; i++)
    {
      double curStepWidth = stepWidth_;
      if (i < nbSteps)
      {
        curStepWidth += extraStepWidth_;
      }
      double theta = (goingToTheRight ? -1. : +1.) * i * stepAngle;
      Eigen::Vector2d supportPoint = targetPose_.pos();
      Eigen::Vector2d lateralVec = {-std::sin(theta), std::cos(theta)};
      lateralVec *= (goingToTheRight ? -1. : +1.) * 0.5 * curStepWidth;
      Eigen::Vector2d leadPos = supportPoint + lateralVec;
      Eigen::Vector2d followPos = supportPoint - lateralVec;
      SE2d leadStepPose = {leadPos.x(), leadPos.y(), theta};
      SE2d followStepPose = {followPos.x(), followPos.y(), theta};

      Contact followStep, leadStep;
      followStep.pose = followStepPose.asPTransform();
      followStep.surfaceName = followFootSurfaceName;
      leadStep.pose = leadStepPose.asPTransform();
      leadStep.surfaceName = leadFootSurfaceName;
      customPlan_.appendContact(leadStep);
      customPlan_.appendContact(followStep);
      nbFootsteps_ += 2;
    }

    stepAngle_ = stepAngle;
    stepLength_ = 0.;
  }

  void PlanInterpolator::updateSupportPath(const sva::PTransformd & X_0_lf, const sva::PTransformd & X_0_rf)
  {
    constexpr double PATH_STEP = 0.05;
    sva::PTransformd X_0_mid = sva::interpolate(X_0_lf, X_0_rf, 0.5);
    supportPathDisplay_.clear();
    for (double s = 0.; s <= 1.; s += PATH_STEP)
    {
      Eigen::Vector2d p = supportPath_.pos(s);
      SE2d pose = {p.x(), p.y(), /* theta = */ 0.};
      sva::PTransformd X_0_p = pose * X_0_mid;
      supportPathDisplay_.push_back(X_0_p.translation());
    }
  }

  void PlanInterpolator::restoreDefaults()
  {
    extraStepWidth_ = DEFAULT_EXTRA_STEP_WIDTH;
    initPose_.theta = 0.; // [rad]
    startWithRightFootstep_ = true;
    supportPath_.extraInitVelScaling(1.);
    supportPath_.extraTargetVelScaling(1.);
  }
}
