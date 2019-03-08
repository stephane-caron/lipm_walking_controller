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

#include <mc_rbdyn/rpy_utils.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/utils/clamp.h>

namespace lipm_walking
{
  namespace
  {
    // The following constants depend on the robot model (here HRP-4)
    constexpr double MAX_CHEST_P = +0.4; // [rad], DOF limit is +0.5 [rad]
    constexpr double MIN_CHEST_P = -0.1; // [rad], DOF limit is -0.2 [rad]
    constexpr char TORSO_BODY_NAME[] = "torso";
  }

  Controller::Controller(std::shared_ptr<mc_rbdyn::RobotModule> robotModule, double dt, const mc_rtc::Configuration & config)
    : mc_control::fsm::Controller(robotModule, dt, config),
      halfSitPose(controlRobot().mbc().q),
      floatingBaseObs_(controlRobot()),
      comVelFilter_(dt, /* cutoff period = */ 0.01),
      netWrenchObs_(dt),
      stabilizer_(controlRobot(), pendulum_, dt)
  {
    // Set up upper-body tasks
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

    double torsoStiffness = config("tasks")("torso")("stiffness");
    double torsoWeight = config("tasks")("torso")("weight");
    config("tasks")("torso")("pitch", defaultTorsoPitch_);
    torsoPitch_ = defaultTorsoPitch_;
    torsoTask = std::make_shared<mc_tasks::OrientationTask>(TORSO_BODY_NAME, robots(), 0);
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
    plans_ = config("plans");
    mpcConfig_ = config("mpc");
    sole_ = config("sole");
    std::string initialPlan = plans_.keys()[0];
    config("initial_plan", initialPlan);
    if (config.has("stabilizer"))
    {
      stabilizer_.configure(config("stabilizer"));
    }

    loadFootstepPlan(initialPlan);
    updateRobotMass(robot().mass());
    stabilizer_.reset(robots());
    stabilizer_.wrenchFaceMatrix(sole_);

    logger().addLogEntry("controlRobot_LeftFoot", [this]() { return controlRobot().surfacePose("LeftFoot"); });
    logger().addLogEntry("controlRobot_LeftFootCenter", [this]() { return controlRobot().surfacePose("LeftFootCenter"); });
    logger().addLogEntry("controlRobot_RightFoot", [this]() { return controlRobot().surfacePose("RightFoot"); });
    logger().addLogEntry("controlRobot_RightFootCenter", [this]() { return controlRobot().surfacePose("RightFootCenter"); });
    logger().addLogEntry("controlRobot_com", [this]() { return controlCom_; });
    logger().addLogEntry("controlRobot_comd", [this]() { return controlComd_; });
    logger().addLogEntry("controlRobot_comd_norm", [this]() { return controlComd_.norm(); });
    logger().addLogEntry("controlRobot_dcm", [this]() -> Eigen::Vector3d { return controlCom_ + controlComd_ / pendulum_.omega(); });
    logger().addLogEntry("controlRobot_posW", [this]() { return controlRobot().posW(); });
    logger().addLogEntry("error_com", [this]() -> Eigen::Vector3d { return controlCom_ - realCom_; });
    logger().addLogEntry("error_comd", [this]() -> Eigen::Vector3d { return controlComd_ - realComd_; });
    logger().addLogEntry("error_dcm", [this]() -> Eigen::Vector3d { return (controlCom_ - realCom_) + (controlComd_ - realComd_) / pendulum_.omega(); });
    logger().addLogEntry("error_zmp", [this]() -> Eigen::Vector3d { return stabilizer_.distribZMP() - netWrenchObs_.zmp(); });
    logger().addLogEntry("mpc_failures", [this]() { return nbMPCFailures_; });
    logger().addLogEntry("mpc_weights_jerk", [this]() { return mpc_.jerkWeight; });
    logger().addLogEntry("mpc_weights_vel", [this]() { return mpc_.velWeights; });
    logger().addLogEntry("mpc_weights_zmp", [this]() { return mpc_.zmpWeight; });
    logger().addLogEntry("left_foot_ratio", [this]() { return leftFootRatio_; });
    logger().addLogEntry("left_foot_ratio_measured", [this]() { return measuredLeftFootRatio(); });
    logger().addLogEntry("pendulum_com", [this]() { return pendulum_.com(); });
    logger().addLogEntry("pendulum_comd", [this]() { return pendulum_.comd(); });
    logger().addLogEntry("pendulum_comdd", [this]() { return pendulum_.comdd(); });
    logger().addLogEntry("pendulum_dcm", [this]() { return pendulum_.dcm(); });
    logger().addLogEntry("pendulum_omega", [this]() { return pendulum_.omega(); });
    logger().addLogEntry("pendulum_zmp", [this]() { return pendulum_.zmp(); });
    logger().addLogEntry("plan_com_height", [this]() { return plan.comHeight(); });
    logger().addLogEntry("plan_double_support_duration", [this]() { return plan.doubleSupportDuration(); });
    logger().addLogEntry("plan_final_dsp_duration", [this]() { return plan.finalDSPDuration(); });
    logger().addLogEntry("plan_init_dsp_duration", [this]() { return plan.initDSPDuration(); });
    logger().addLogEntry("plan_landing_pitch", [this]() { return plan.landingPitch(); });
    logger().addLogEntry("plan_landing_ratio", [this]() { return plan.landingRatio(); });
    logger().addLogEntry("plan_ref_vel", [this]() { return plan.supportContact().refVel; });
    logger().addLogEntry("plan_single_support_duration", [this]() { return plan.singleSupportDuration(); });
    logger().addLogEntry("plan_swing_height", [this]() { return plan.swingHeight(); });
    logger().addLogEntry("plan_takeoff_offset", [this]() { return plan.takeoffOffset(); });
    logger().addLogEntry("plan_takeoff_pitch", [this]() { return plan.takeoffPitch(); });
    logger().addLogEntry("plan_takeoff_ratio", [this]() { return plan.takeoffRatio(); });
    logger().addLogEntry("realRobot_LeftFoot", [this]() { return realRobot().surfacePose("LeftFoot"); });
    logger().addLogEntry("realRobot_LeftFootCenter", [this]() { return realRobot().surfacePose("LeftFootCenter"); });
    logger().addLogEntry("realRobot_RightFoot", [this]() { return realRobot().surfacePose("RightFoot"); });
    logger().addLogEntry("realRobot_RightFootCenter", [this]() { return realRobot().surfacePose("RightFootCenter"); });
    logger().addLogEntry("realRobot_com", [this]() { return realCom_; });
    logger().addLogEntry("realRobot_comd", [this]() { return realComd_; });
    logger().addLogEntry("realRobot_dcm", [this]() -> Eigen::Vector3d { return realCom_ + realComd_ / pendulum_.omega(); });
    logger().addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });
    logger().addLogEntry("realRobot_wrench", [this]() { return netWrenchObs_.wrench(); });
    logger().addLogEntry("realRobot_zmp", [this]() { return netWrenchObs_.zmp(); });
    stabilizer_.addLogEntries(logger());

    if (gui_)
    {
      using namespace mc_rtc::gui;
      gui_->addElement(
        {"Walking", "Controller"},
        Button("# EMERGENCY STOP",
          [this]()
          {
            emergencyStop = true;
            this->interrupt();
          }),
        Button("Reset",
          [this]() { this->resume("Initial"); }));
      gui_->addElement({"Walking", "Advanced"},
        Label("Mass [kg]",
          [this]() { return std::round(robotMass_ * 100.) / 100.; }),
        Label("Torso pitch [rad]",
          [this]() { return torsoPitch_; }),
        NumberInput("Velocity cutoff period [s]",
          [this]() { return comVelFilter_.cutoffPeriod(); },
          [this](double T) { comVelFilter_.cutoffPeriod(T); }),
        NumberInput("Torso pitch override [rad]",
          [this]() { return defaultTorsoPitch_; },
          [this](double pitch)
          {
            pitch = clamp(pitch, MIN_CHEST_P, MAX_CHEST_P);
            defaultTorsoPitch_ = pitch;
            torsoPitch_ = pitch;
          }));
      gui_->addElement(
        {"Walking", "Plan"},
        Label(
          "Name",
          [this]() { return plan.name; }),
        NumberInput(
          "CoM height",
          [this]() { return plan.comHeight(); },
          [this](double height) { plan.comHeight(height); }),
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
          [this](double duration) { plan.finalDSPDuration(duration); }),
        NumberInput(
          "Swing height [m]",
          [this]() { return plan.swingHeight(); },
          [this](double height) { plan.swingHeight(height); }),
        NumberInput(
          "Takeoff pitch [rad]",
          [this]() { return plan.takeoffPitch(); },
          [this](double pitch) { plan.takeoffPitch(pitch); }),
        NumberInput(
          "Landing pitch [rad]",
          [this]() { return plan.landingPitch(); },
          [this](double pitch) { plan.landingPitch(pitch); }),
        NumberInput(
          "Takeoff ratio",
          [this]() { return plan.takeoffRatio(); },
          [this](double ratio) { plan.takeoffRatio(ratio); }),
        NumberInput(
          "Landing ratio",
          [this]() { return plan.landingRatio(); },
          [this](double ratio) { plan.landingRatio(ratio); }));
      mpc_.addGUIElements(gui_);
      stabilizer_.addGUIElements(gui_);
    }

    LOG_SUCCESS("LIPMWalking controller init done " << this)
  }

  void Controller::updateRobotMass(double mass)
  {
    robotMass_ = mass;
    stabilizer_.mass(mass);
    LOG_INFO("Robot mass updated to " << mass << " [kg]");
  }

  void Controller::internalReset()
  {
    if(!isSpinning_)
    {
      spinThread_ = std::thread(std::bind(&Controller::spinner, this));
      isSpinning_ = true;
    }

    auto X_0_fb = plan.computeInitialTransform(controlRobot());
    controlRobot().posW(X_0_fb);
    controlRobot().setBaseLinkVelocity(Eigen::Vector6d::Zero());
    realRobot().posW(X_0_fb);
    realRobot().setBaseLinkVelocity(Eigen::Vector6d::Zero());
    floatingBaseObs_.reset(X_0_fb);

    Eigen::Vector3d initCom = controlRobot().com();

    comVelFilter_.reset(initCom);

    netWrenchObs_.update(realRobot(), supportContact());
    pendulum_.reset(initCom);
    plan.reset();

    // reset solver tasks
    postureTask->posture(halfSitPose);
    solver().removeTask(pelvisTask);
    solver().removeTask(torsoTask);
    stabilizer_.reset(robots());

    stabilizer_.updateState(initCom, Eigen::Vector3d::Zero(), sva::ForceVecd{Eigen::Vector6d::Zero()});

    controlCom_ = initCom;
    controlComd_ = Eigen::Vector3d::Zero();
    leftFootRatioJumped_ = false;
    leftFootRatio_ = 0.5;
    nbMPCFailures_ = 0;
    pauseWalking = false;
    realCom_ = initCom; // realRobot() may not be initialized yet
    realComd_ = Eigen::Vector3d::Zero();

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
    if (!mc_control::fsm::Controller::running())
    {
      return mc_control::fsm::Controller::run();
    }

    controlCom_ = controlRobot().com();
    controlComd_ = controlRobot().comVelocity();
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
    plan = plans_(name);
    plan.name = name;
    plan.reset();
    mpc_.configure(mpcConfig_);
    if (!plan.mpcConfig.empty())
    {
      mpc_.configure(plan.mpcConfig);
    }
    plan.complete(sole_);
    torsoPitch_ = (plan.hasTorsoPitch()) ? plan.torsoPitch() : defaultTorsoPitch_;
    LOG_INFO("Loaded footstep plan \"" << name << "\"");
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
}

CONTROLLER_CONSTRUCTOR("LIPMWalking", lipm_walking::Controller)
