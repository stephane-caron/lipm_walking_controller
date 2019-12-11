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

#include <mc_rbdyn/rpy_utils.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/utils/clamp.h>

namespace lipm_walking
{
  Controller::Controller(std::shared_ptr<mc_rbdyn::RobotModule> robotModule, double dt, const mc_rtc::Configuration & config)
    : mc_control::fsm::Controller(robotModule, dt, config),
      planInterpolator(gui()),
      halfSitPose(controlRobot().mbc().q),
      floatingBaseObs_(controlRobot()),
      comVelFilter_(dt, /* cutoff period = */ 0.01),
      netWrenchObs_(),
      stabilizer_(controlRobot(), pendulum_, dt)
  {
    auto robotConfig = config("robot_models")(controlRobot().name());
    auto planConfig = config("plans")(controlRobot().name());

    // Patch CoM height and step width in all plans
    std::vector<std::string> plans = planConfig.keys();
    double comHeight = robotConfig("com")("height");
    maxCoMHeight_ = robotConfig("com")("max_height");
    minCoMHeight_ = robotConfig("com")("min_height");
    maxTorsoPitch_ = robotConfig("torso")("max_pitch");
    minTorsoPitch_ = robotConfig("torso")("min_pitch");
    for (const auto & p : plans)
    {
      auto plan = planConfig(p);
      if (!plan.has("com_height"))
      {
        plan.add("com_height", comHeight);
      }
    }

    // Add upper-body tasks
    double pelvisStiffness = config("tasks")("pelvis")("stiffness");
    double pelvisWeight = config("tasks")("pelvis")("weight");
    std::string pelvisBodyName = robot().mb().body(0).name();
    pelvisTask = std::make_shared<mc_tasks::OrientationTask>(pelvisBodyName, robots(), 0);
    pelvisTask->orientation(pelvisOrientation_);
    pelvisTask->stiffness(pelvisStiffness);
    pelvisTask->weight(pelvisWeight);

    double postureStiffness = config("tasks")("posture")("stiffness");
    double postureWeight = config("tasks")("posture")("weight");
    postureTask = getPostureTask(robot().name());
    postureTask->stiffness(postureStiffness);
    postureTask->weight(postureWeight);

    std::string torsoName = robotConfig("torso")("name");
    robotConfig("torso")("pitch", defaultTorsoPitch_);
    double torsoStiffness = config("tasks")("torso")("stiffness");
    double torsoWeight = config("tasks")("torso")("weight");
    torsoPitch_ = defaultTorsoPitch_;
    torsoTask = std::make_shared<mc_tasks::OrientationTask>(torsoName, robots(), 0);
    torsoTask->orientation(mc_rbdyn::rpyToMat({0, torsoPitch_, 0}) * pelvisOrientation_);
    torsoTask->stiffness(torsoStiffness);
    torsoTask->weight(torsoWeight);

    // Set half-sitting pose for posture task
    const auto & halfSit = robotModule->stance();
    const auto & refJointOrder = robot().refJointOrder();
    for (unsigned i = 0; i < refJointOrder.size(); ++i)
    {
      if (robot().hasJoint(refJointOrder[i]))
      {
        halfSitPose[robot().jointIndexByName(refJointOrder[i])] = halfSit.at(refJointOrder[i]);
      }
    }

    // Read settings from configuration file
    plans_ = planConfig;
    mpcConfig_ = config("mpc");
    std::string initialPlan = plans_.keys()[0];
    config("initial_plan", initialPlan);

    // Read sole properties from robot model and configuration file
    sva::PTransformd X_0_lfc = controlRobot().surfacePose("LeftFootCenter");
    sva::PTransformd X_0_rfc = controlRobot().surfacePose("RightFootCenter");
    sva::PTransformd X_0_lf = controlRobot().surfacePose("LeftFoot");
    sva::PTransformd X_lfc_lf = X_0_lf * X_0_lfc.inv();
    sva::PTransformd X_rfc_lfc = X_0_lfc * X_0_rfc.inv();
    double stepWidth = X_rfc_lfc.translation().y();
    sole_ = robotConfig("sole");
    sole_.leftAnkleOffset = X_lfc_lf.translation().head<2>();

    std::vector<std::string> comActiveJoints = robotConfig("com")("active_joints");
    config("stabilizer").add("admittance", robotConfig("admittance"));
    config("stabilizer").add("dcm_tracking", robotConfig("dcm_tracking"));
    config("stabilizer")("tasks")("com").add("active_joints", comActiveJoints);
    stabilizer_.configure(config("stabilizer"));

    planInterpolator.configure(config("plans"));
    planInterpolator.stepWidth(stepWidth);
    loadFootstepPlan(initialPlan);
    mpc_.sole(sole_);
    stabilizer_.reset(robots());
    stabilizer_.sole(sole_);
    stabilizer_.wrenchFaceMatrix(sole_);

    addLogEntries(logger());
    mpc_.addLogEntries(logger());
    stabilizer_.addLogEntries(logger());

    if (gui_)
    {
      addGUIElements(gui_);
      mpc_.addGUIElements(gui_);
      stabilizer_.addGUIElements(gui_);
    }

    LOG_SUCCESS("LIPMWalking controller init done " << this)
  }

  void Controller::addLogEntries(mc_rtc::Logger & logger)
  {
    logger.addLogEntry("controlRobot_LeftFoot", [this]() { return controlRobot().surfacePose("LeftFoot"); });
    logger.addLogEntry("controlRobot_LeftFootCenter", [this]() { return controlRobot().surfacePose("LeftFootCenter"); });
    logger.addLogEntry("controlRobot_RightFoot", [this]() { return controlRobot().surfacePose("RightFoot"); });
    logger.addLogEntry("controlRobot_RightFootCenter", [this]() { return controlRobot().surfacePose("RightFootCenter"); });
    logger.addLogEntry("controlRobot_com", [this]() { return controlRobot().com(); });
    logger.addLogEntry("controlRobot_comd", [this]() { return controlRobot().comVelocity(); });
    logger.addLogEntry("controlRobot_posW", [this]() { return controlRobot().posW(); });
    logger.addLogEntry("mpc_failures", [this]() { return nbMPCFailures_; });
    logger.addLogEntry("left_foot_ratio", [this]() { return leftFootRatio_; });
    logger.addLogEntry("left_foot_ratio_measured", [this]() { return measuredLeftFootRatio(); });
    logger.addLogEntry("pendulum_com", [this]() { return pendulum_.com(); });
    logger.addLogEntry("pendulum_comd", [this]() { return pendulum_.comd(); });
    logger.addLogEntry("pendulum_comdd", [this]() { return pendulum_.comdd(); });
    logger.addLogEntry("pendulum_dcm", [this]() { return pendulum_.dcm(); });
    logger.addLogEntry("pendulum_omega", [this]() { return pendulum_.omega(); });
    logger.addLogEntry("pendulum_zmp", [this]() { return pendulum_.zmp(); });
    logger.addLogEntry("plan_com_height", [this]() { return plan.comHeight(); });
    logger.addLogEntry("plan_double_support_duration", [this]() { return plan.doubleSupportDuration(); });
    logger.addLogEntry("plan_final_dsp_duration", [this]() { return plan.finalDSPDuration(); });
    logger.addLogEntry("plan_init_dsp_duration", [this]() { return plan.initDSPDuration(); });
    logger.addLogEntry("plan_landing_duration", [this]() { return plan.landingDuration(); });
    logger.addLogEntry("plan_landing_pitch", [this]() { return plan.landingPitch(); });
    logger.addLogEntry("plan_ref_vel", [this]() { return plan.supportContact().refVel; });
    logger.addLogEntry("plan_single_support_duration", [this]() { return plan.singleSupportDuration(); });
    logger.addLogEntry("plan_swing_height", [this]() { return plan.swingHeight(); });
    logger.addLogEntry("plan_takeoff_duration", [this]() { return plan.takeoffDuration(); });
    logger.addLogEntry("plan_takeoff_offset", [this]() { return plan.takeoffOffset(); });
    logger.addLogEntry("plan_takeoff_pitch", [this]() { return plan.takeoffPitch(); });
    logger.addLogEntry("realRobot_LeftFoot", [this]() { return realRobot().surfacePose("LeftFoot"); });
    logger.addLogEntry("realRobot_LeftFootCenter", [this]() { return realRobot().surfacePose("LeftFootCenter"); });
    logger.addLogEntry("realRobot_RightFoot", [this]() { return realRobot().surfacePose("RightFoot"); });
    logger.addLogEntry("realRobot_RightFootCenter", [this]() { return realRobot().surfacePose("RightFootCenter"); });
    logger.addLogEntry("realRobot_com", [this]() { return realCom_; });
    logger.addLogEntry("realRobot_comd", [this]() { return realComd_; });
    logger.addLogEntry("realRobot_dcm", [this]() -> Eigen::Vector3d { return realCom_ + realComd_ / pendulum_.omega(); });
    logger.addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });
    logger.addLogEntry("realRobot_wrench", [this]() { return netWrenchObs_.wrench(); });
    logger.addLogEntry("realRobot_zmp", [this]() { return netWrenchObs_.zmp(); });
  }

  void Controller::addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
  {
    using namespace mc_rtc::gui;

    gui->addElement(
      {"Walking", "Controller"},
      Button(
        "# EMERGENCY STOP",
        [this]()
        {
          emergencyStop = true;
          this->interrupt();
        }),
      Button(
        "Reset",
        [this]() { this->resume("Initial"); }));

    gui->addElement(
      {"Walking", "Phases"},
      Label(
        "Plan name",
        [this]() { return plan.name; }),
      NumberInput(
        "Initial DSP duration [s]",
        [this]() { return plan.initDSPDuration(); },
        [this](double duration) { plan.initDSPDuration(duration); }),
      NumberInput(
        "SSP duration [s]",
        [this]() { return plan.singleSupportDuration(); },
        [this](double duration)
        {
          constexpr double T = ModelPredictiveControl::SAMPLING_PERIOD;
          duration = std::round(duration / T) * T;
          plan.singleSupportDuration(duration);
        }),
      NumberInput(
        "DSP duration [s]",
        [this]() { return plan.doubleSupportDuration(); },
        [this](double duration)
        {
          constexpr double T = ModelPredictiveControl::SAMPLING_PERIOD;
          duration = std::round(duration / T) * T;
          plan.doubleSupportDuration(duration);
        }),
      NumberInput(
        "Final DSP duration [s]",
        [this]() { return plan.finalDSPDuration(); },
        [this](double duration) { plan.finalDSPDuration(duration); }));

    gui->addElement(
      {"Walking", "CoM"},
      Label(
        "Plan name",
        [this]() { return plan.name; }),
      NumberInput(
          "CoM height",
          [this]() { return plan.comHeight(); },
          [this](double height)
          {
            height = clamp(height, minCoMHeight_, maxCoMHeight_);
            plan.comHeight(height);
          }),
      NumberInput(
        "Torso pitch [rad]",
        [this]() { return torsoPitch_; },
        [this](double pitch)
        {
          pitch = clamp(pitch, minTorsoPitch_, maxTorsoPitch_);
          defaultTorsoPitch_ = pitch;
          torsoPitch_ = pitch;
        }));

    gui->addElement(
      {"Walking", "Swing foot"},
      Label(
        "Plan name",
        [this]() { return plan.name; }),
      NumberInput(
        "Swing height [m]",
        [this]() { return plan.swingHeight(); },
        [this](double height) { plan.swingHeight(height); }),
      NumberInput(
        "Takeoff duration",
        [this]() { return plan.takeoffDuration(); },
        [this](double duration) { plan.takeoffDuration(duration); }),
      NumberInput(
        "Takeoff pitch [rad]",
        [this]() { return plan.takeoffPitch(); },
        [this](double pitch) { plan.takeoffPitch(pitch); }),
      NumberInput(
        "Landing duration",
        [this]() { return plan.landingDuration(); },
        [this](double duration) { plan.landingDuration(duration); }),
      NumberInput(
        "Landing pitch [rad]",
        [this]() { return plan.landingPitch(); },
        [this](double pitch) { plan.landingPitch(pitch); }));

    constexpr double ARROW_HEAD_DIAM = 0.015;
    constexpr double ARROW_HEAD_LEN = 0.05;
    constexpr double ARROW_SHAFT_DIAM = 0.015;
    constexpr double FORCE_SCALE = 0.0015;

    const std::map<char, Color> COLORS =
    {
      {'r', Color{1.0, 0.0, 0.0}},
      {'g', Color{0.0, 1.0, 0.0}},
      {'b', Color{0.0, 0.0, 1.0}},
      {'y', Color{1.0, 0.5, 0.0}},
      {'c', Color{0.0, 0.5, 1.0}},
      {'m', Color{1.0, 0.0, 0.5}}
    };

    ArrowConfig pendulumArrowConfig;
    pendulumArrowConfig.color = COLORS.at('y');
    pendulumArrowConfig.end_point_scale = 0.02;
    pendulumArrowConfig.head_diam = .1 * ARROW_HEAD_DIAM;
    pendulumArrowConfig.head_len = .1 * ARROW_HEAD_LEN;
    pendulumArrowConfig.scale = 1.;
    pendulumArrowConfig.shaft_diam = .1 * ARROW_SHAFT_DIAM;
    pendulumArrowConfig.start_point_scale = 0.02;

    ArrowConfig pendulumForceArrowConfig;
    pendulumForceArrowConfig.shaft_diam = 1 * ARROW_SHAFT_DIAM;
    pendulumForceArrowConfig.head_diam = 1 * ARROW_HEAD_DIAM;
    pendulumForceArrowConfig.head_len = 1 * ARROW_HEAD_LEN;
    pendulumForceArrowConfig.scale = 1.;
    pendulumForceArrowConfig.start_point_scale = 0.02;
    pendulumForceArrowConfig.end_point_scale = 0.;

    ArrowConfig netWrenchForceArrowConfig = pendulumForceArrowConfig;
    netWrenchForceArrowConfig.color = COLORS.at('r');

    ArrowConfig refPendulumForceArrowConfig = pendulumForceArrowConfig;
    refPendulumForceArrowConfig = COLORS.at('y');

    ForceConfig copForceConfig(COLORS.at('g'));
    copForceConfig.start_point_scale = 0.02;
    copForceConfig.end_point_scale = 0.;

    auto footStepPolygon = [](const Contact& contact)
    {
      std::vector<Eigen::Vector3d> polygon;
      polygon.push_back(contact.vertex0());
      polygon.push_back(contact.vertex1());
      polygon.push_back(contact.vertex2());
      polygon.push_back(contact.vertex3());
      return polygon;
    };

    gui->addElement(
      {"Walking", "Markers", "Contacts"},
      Polygon(
        "SupportContact",
        COLORS.at('g'),
        [this, footStepPolygon]()
        {
          return footStepPolygon(supportContact());
        }),
      Polygon(
        "TargetContact",
        COLORS.at('b'),
        [this, footStepPolygon]()
        {
          return footStepPolygon(targetContact());
        }),
      Polygon(
        "FootstepPlan",
        COLORS.at('b'),
        [this, footStepPolygon]()
        {
          std::vector<std::vector<Eigen::Vector3d>> polygons;
          const auto & contacts = plan.contacts();
          for (unsigned i = 0; i < contacts.size(); i++)
          {
            auto & contact = contacts[i];
            double supportDist = (contact.p() - supportContact().p()).norm();
            double targetDist = (contact.p() - targetContact().p()).norm();
            constexpr double SAME_CONTACT_DIST = 0.005;
            // only display contact if it is not the support or target contact
            if (supportDist > SAME_CONTACT_DIST && targetDist > SAME_CONTACT_DIST)
            {
              polygons.push_back(footStepPolygon(contact));
            }
          }
          return polygons;
        }));

    constexpr double COM_POINT_SIZE = 0.02;
    constexpr double DCM_POINT_SIZE = 0.015;

    gui->addElement(
      {"Walking", "Markers", "CoM-DCM"},
      Arrow(
        "Pendulum_CoM",
        pendulumArrowConfig,
        [this]() -> Eigen::Vector3d
        {
          return this->pendulum_.zmp();
        },
        [this]() -> Eigen::Vector3d
        {
          return this->pendulum_.com();
        }),
      Point3D(
        "Measured_CoM",
        PointConfig(COLORS.at('g'), COM_POINT_SIZE),
        [this]()
        {
          return realCom_;
        }),
      Point3D(
        "Pendulum_DCM",
        PointConfig(COLORS.at('y'), DCM_POINT_SIZE),
        [this]()
        {
          return pendulum_.dcm();
        }),
      Point3D(
        "Measured_DCM",
        PointConfig(COLORS.at('g'), DCM_POINT_SIZE),
        [this]() -> Eigen::Vector3d
        {
          return realCom_ + realComd_ / pendulum().omega();
        }));

    gui->addElement(
      {"Walking", "Markers", "Net wrench"},
      Point3D(
        "Stabilizer_ZMP",
        PointConfig(COLORS.at('m'), 0.02),
        [this]() { return stabilizer_.zmp(); }),
      Point3D(
        "Measured_ZMP",
        PointConfig(COLORS.at('r'), 0.02),
        [this]() -> Eigen::Vector3d
        {
          return netWrenchObs_.zmp();
        }),
      Arrow(
        "Measured_ZMPForce",
        netWrenchForceArrowConfig,
        [this]() -> Eigen::Vector3d
        {
          return this->netWrenchObs_.zmp();
        },
        [this, FORCE_SCALE]() -> Eigen::Vector3d
        {
          return netWrenchObs_.zmp() + FORCE_SCALE * netWrenchObs_.wrench().force();
        }));

    gui->addElement(
      {"Walking", "Markers", "Foot wrenches"},
      Point3D(
        "Stabilizer_LeftCoP",
        PointConfig(COLORS.at('m'), 0.01),
        [this]() { return stabilizer_.leftFootTask->targetCoPW(); }),
      Force(
        "Measured_LeftCoPForce",
        copForceConfig,
        [this]()
        {
          return this->robot().surfaceWrench("LeftFootCenter");
        },
        [this]()
        {
          return sva::PTransformd(this->robot().copW("LeftFootCenter"));
        }),
      Point3D(
        "Stabilizer_RightCoP",
        PointConfig(COLORS.at('m'), 0.01),
        [this]() { return stabilizer_.rightFootTask->targetCoPW(); }),
      Force(
        "Measured_RightCoPForce",
        copForceConfig,
        [this]()
        {
          return this->robot().surfaceWrench("RightFootCenter");
        },
        [this]()
        {
          return sva::PTransformd(this->robot().copW("RightFootCenter"));
        }));
  }

  void Controller::reset(const mc_control::ControllerResetData & data)
  {
    mc_control::fsm::Controller::reset(data);
    if (gui_)
    {
      gui_->removeCategory({"Contacts"});
    }
  }

  void Controller::internalReset()
  {
    // (1) update floating-base transforms of both robot mbc's
    auto X_0_fb = supportContact().robotTransform(controlRobot());
    controlRobot().posW(X_0_fb);
    controlRobot().velW(sva::MotionVecd::Zero());
    realRobot().posW(X_0_fb);
    realRobot().velW(sva::MotionVecd::Zero());

    // (2) update contact frames to coincide with surface ones
    loadFootstepPlan(plan.name);

    // (3) reset solver tasks
    postureTask->posture(halfSitPose);
    solver().removeTask(pelvisTask);
    solver().removeTask(torsoTask);
    stabilizer_.reset(robots());

    // (4) reset controller attributes
    leftFootRatioJumped_ = true;
    leftFootRatio_ = 0.5;
    nbMPCFailures_ = 0;
    pauseWalking = false;
    pauseWalkingRequested = false;

    Eigen::Vector3d controlCoM = controlRobot().com();
    comVelFilter_.reset(controlCoM);
    pendulum_.reset(controlCoM);

    // (5) reset floating-base observers
    floatingBaseObs_.reset(controlRobot().posW());
    floatingBaseObs_.leftFootRatio(leftFootRatio_);
    floatingBaseObs_.run(realRobot());
    updateRealFromKinematics(); // after leftFootRatio_ is initialized

    // (6) updates that depend on realCom_
    netWrenchObs_.update(realRobot(), supportContact());
    stabilizer_.updateState(realCom_, realComd_, netWrenchObs_.wrench(), leftFootRatio_);

    stopLogSegment();
  }

  void Controller::leftFootRatio(double ratio)
  {
    double maxRatioVar = 1.5 * timeStep / plan.doubleSupportDuration();
    if (std::abs(ratio - leftFootRatio_) > maxRatioVar)
    {
      LOG_WARNING("Left foot ratio jumped from " << leftFootRatio_ << " to " << ratio);
      leftFootRatioJumped_ = true;
    }
    leftFootRatio_ = clamp(ratio, 0., 1., "leftFootRatio");
  }

  bool Controller::run()
  {
    if (emergencyStop)
    {
      return false;
    }
    if (pauseWalkingRequested)
    {
      pauseWalkingCallback();
    }
    if (!mc_control::fsm::Controller::running())
    {
      return mc_control::fsm::Controller::run();
    }

    ctlTime_ += timeStep;

    warnIfRobotIsInTheAir();

    floatingBaseObs_.leftFootRatio(leftFootRatio_);
    floatingBaseObs_.run(realRobot());
    updateRealFromKinematics();
    sva::PTransformd X_0_a = floatingBaseObs_.getAnchorFrame(controlRobot());
    pelvisOrientation_ = X_0_a.rotation();
    pelvisTask->orientation(pelvisOrientation_);
    torsoTask->orientation(mc_rbdyn::rpyToMat({0, torsoPitch_, 0}) * pelvisOrientation_);

    netWrenchObs_.update(realRobot(), supportContact());
    stabilizer_.updateState(realCom_, realComd_, netWrenchObs_.wrench(), leftFootRatio_);

    bool ret = mc_control::fsm::Controller::run();
    if (mc_control::fsm::Controller::running())
    {
      postureTask->posture(halfSitPose); // reset posture in case the FSM updated it
    }
    return ret;
  }

  void Controller::pauseWalkingCallback(bool verbose)
  {
    constexpr double MAX_HEIGHT_DIFF = 0.02; // [m]
    if (pauseWalking)
    {
      LOG_WARNING("Already pausing, how did you get there?");
      return;
    }
    else if (std::abs(supportContact().z() - targetContact().z()) > MAX_HEIGHT_DIFF)
    {
      if (!pauseWalkingRequested || verbose)
      {
        LOG_WARNING("Cannot pause on uneven ground, will pause later");
      }
      gui()->removeElement({"Walking", "Controller"}, "Pause walking");
      pauseWalkingRequested = true;
    }
    else if (pauseWalkingRequested)
    {
      LOG_WARNING("Pausing now that contacts are at same level");
      pauseWalkingRequested = false;
      pauseWalking = true;
    }
    else // (!pauseWalkingRequested)
    {
      gui()->removeElement({"Walking", "Controller"}, "Pause walking");
      pauseWalking = true;
    }
  }

  void Controller::warnIfRobotIsInTheAir()
  {
    static bool isInTheAir = false;
    constexpr double CONTACT_THRESHOLD = 30.; // [N]
    double leftFootPressure = realRobot().forceSensor("LeftFootForceSensor").force().z();
    double rightFootPressure = realRobot().forceSensor("RightFootForceSensor").force().z();
    if (leftFootPressure < CONTACT_THRESHOLD && rightFootPressure < CONTACT_THRESHOLD)
    {
      if (!isInTheAir)
      {
        LOG_WARNING("Robot is in the air");
        isInTheAir = true;
      }
    }
    else
    {
      if (isInTheAir)
      {
        LOG_INFO("Robot is on the ground again");
        isInTheAir = false;
      }
    }
  }

  void Controller::updateRealFromKinematics()
  {
    floatingBaseObs_.updateRobot(realRobot());
    realCom_ = realRobot().com();
    if (!leftFootRatioJumped_)
    {
      comVelFilter_.update(realCom_);
    }
    else // don't update velocity when CoM position jumped
    {
      comVelFilter_.updatePositionOnly(realCom_);
      leftFootRatioJumped_ = false;
    }
    realComd_ = comVelFilter_.vel();
  }

  void Controller::loadFootstepPlan(std::string name)
  {
    double initHeight = (plan.name.length() > 0) ? plan.supportContact().p().z() : 0.;

    plan = plans_(name);
    plan.name = name;
    mpc_.configure(mpcConfig_);
    if (!plan.mpcConfig.empty())
    {
      mpc_.configure(plan.mpcConfig);
    }
    plan.complete(sole_);
    const sva::PTransformd & X_0_lf = controlRobot().surfacePose("LeftFootCenter");
    const sva::PTransformd & X_0_rf = controlRobot().surfacePose("RightFootCenter");
    plan.updateInitialTransform(X_0_lf, X_0_rf, initHeight);
    plan.rewind();
    torsoPitch_ = (plan.hasTorsoPitch()) ? plan.torsoPitch() : defaultTorsoPitch_;
    LOG_INFO("Loaded footstep plan \"" << name << "\"");
  }

  void Controller::startLogSegment(const std::string & label)
  {
    if (segmentName_.length() > 0)
    {
      stopLogSegment();
    }
    segmentName_ = "t_" + std::to_string(++nbLogSegments_).erase(0, 1) + "_" + label;
    logger().addLogEntry(segmentName_, [this]() { return ctlTime_; });
  }

  void Controller::stopLogSegment()
  {
    logger().removeLogEntry(segmentName_);
    segmentName_ = "";
  }

  bool Controller::updatePreview()
  {
    mpc_.initState(pendulum());
    mpc_.comHeight(plan.comHeight());
    if (mpc_.solve())
    {
      preview = mpc_.solution();
      return true;
    }
    else
    {
      nbMPCFailures_++;
      return false;
    }
  }
}

CONTROLLER_CONSTRUCTOR("LIPMWalking", lipm_walking::Controller)
