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

#pragma once

#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <lipm_walking/Contact.h>
#include <lipm_walking/FloatingBaseObserver.h>
#include <lipm_walking/FootstepPlan.h>
#include <lipm_walking/ModelPredictiveControl.h>
#include <lipm_walking/NetWrenchObserver.h>
#include <lipm_walking/Pendulum.h>
#include <lipm_walking/PlanInterpolator.h>
#include <lipm_walking/Sole.h>
#include <lipm_walking/Stabilizer.h>
#include <lipm_walking/utils/LowPassVelocityFilter.h>
#include <mutex>
#include <thread>

/** Main controller namespace.
 *
 */
namespace lipm_walking
{

/** Preview update period, same as MPC sampling period.
 *
 */
constexpr double PREVIEW_UPDATE_PERIOD = ModelPredictiveControl::SAMPLING_PERIOD;

/** Main controller class.
 *
 */
struct MC_CONTROL_DLLAPI Controller : public mc_control::fsm::Controller
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Initialize the controller.
   *
   * \param robot Robot model.
   *
   * \param dt Control timestep.
   *
   * \param config Configuration dictionary.
   *
   */
  Controller(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt, const mc_rtc::Configuration & config);

  /** Reset controller.
   *
   * \param data Reset data.
   *
   */
  void reset(const mc_control::ControllerResetData & data) override;

  /** Add GUI panel.
   *
   * \param gui GUI handle.
   *
   */
  void addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);

  /** Add GUI markers.
   *
   * \param gui GUI handle.
   *
   */
  void addGUIMarkers(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);

  /** Log controller entries.
   *
   * \param logger Logger.
   *
   */
  void addLogEntries(mc_rtc::Logger & logger);

  /** Reset robot to its initial (half-sitting) configuration.
   *
   * The reason why I do it inside the controller rather than via the current
   * mc_rtc way (switching to half_sitting controller then back to this one)
   * is <https://gite.lirmm.fr/multi-contact/mc_rtc/issues/54>.
   *
   */
  void internalReset();

  /** Set fraction of total weight that should be sustained by the left foot.
   *
   * \param ratio Number between 0 and 1.
   *
   */
  void leftFootRatio(double ratio);

  /** Load footstep plan from configuration.
   *
   * \param name Plan name.
   *
   */
  void loadFootstepPlan(std::string name);

  /** Callback function called by "Pause walking" button.
   *
   * \param verbose Talk to user on the command line.
   *
   */
  void pauseWalkingCallback(bool verbose = false);

  /** Main function of the controller, called at every control cycle.
   *
   */
  virtual bool run() override;

  /** Start new log segment.
   *
   * \param label Segment label.
   *
   */
  void startLogSegment(const std::string & label);

  /** Stop current log segment.
   *
   */
  void stopLogSegment();

  /** Update horizontal MPC preview.
   *
   */
  bool updatePreview();

  /** Update measured robot's floating base from kinematic observer.
   *
   */
  void updateRealFromKinematics();

  /** Log a warning message when robot is in the air.
   *
   */
  void warnIfRobotIsInTheAir();

  /** Get control robot state.
   *
   */
  mc_rbdyn::Robot & controlRobot()
  {
    return mc_control::fsm::Controller::robot();
  }

  /** Get next double support duration.
   *
   */
  double doubleSupportDuration()
  {
    double duration;
    if(doubleSupportDurationOverride_ > 0.)
    {
      duration = doubleSupportDurationOverride_;
      doubleSupportDurationOverride_ = -1.;
    }
    else
    {
      duration = plan.doubleSupportDuration();
    }
    return duration;
  }

  /** True after the last step.
   *
   */
  bool isLastDSP()
  {
    return (supportContact().id > targetContact().id);
  }

  /** True during the last step.
   *
   */
  bool isLastSSP()
  {
    return (targetContact().id > nextContact().id);
  }

  /** Get fraction of total weight that should be sustained by the left foot.
   *
   */
  double leftFootRatio()
  {
    return leftFootRatio_;
  }

  /** Estimate left foot pressure ratio from force sensors.
   *
   */
  double measuredLeftFootRatio()
  {
    double leftFootPressure = realRobot().forceSensor("LeftFootForceSensor").force().z();
    double rightFootPressure = realRobot().forceSensor("RightFootForceSensor").force().z();
    leftFootPressure = std::max(0., leftFootPressure);
    rightFootPressure = std::max(0., rightFootPressure);
    return leftFootPressure / (leftFootPressure + rightFootPressure);
  }

  /** Get model predictive control solver.
   *
   */
  ModelPredictiveControl & mpc()
  {
    return mpc_;
  }

  /** Get next contact in plan.
   *
   */
  const Contact & nextContact() const
  {
    return plan.nextContact();
  }

  /** Override next DSP duration.
   *
   * \param duration Custom DSP duration.
   *
   */
  void nextDoubleSupportDuration(double duration)
  {
    doubleSupportDurationOverride_ = duration;
  }

  /** This getter is only used for consistency with the rest of mc_rtc.
   *
   */
  Pendulum & pendulum()
  {
    return pendulum_;
  }

  /** Get previous contact in plan.
   *
   */
  const Contact & prevContact() const
  {
    return plan.prevContact();
  }

  /** Get observed robot state.
   *
   */
  mc_rbdyn::Robot & realRobot()
  {
    return real_robots->robot();
  }

  /** Get next SSP duration.
   *
   */
  double singleSupportDuration()
  {
    return plan.singleSupportDuration();
  }

  /** Get model sole properties.
   *
   */
  const Sole & sole() const
  {
    return sole_;
  }

  /** This getter is only used for consistency with the rest of mc_rtc.
   *
   */
  Stabilizer & stabilizer()
  {
    return stabilizer_;
  }

  /** Get current support contact.
   *
   */
  const Contact & supportContact()
  {
    return plan.supportContact();
  }

  /** Get current target contact.
   *
   */
  const Contact & targetContact()
  {
    return plan.targetContact();
  }

public: /* visible to FSM states */
  FootstepPlan plan; /**< Current footstep plan */
  PlanInterpolator planInterpolator; /**< Footstep plan interpolator */
  bool emergencyStop = false; /**< Emergency flag: if on, the controller stops doing anything */
  bool pauseWalking = false; /**< Is the pause-walking behavior engaged? */
  bool pauseWalkingRequested = false; /**< Has user clicked on the "Pause walking" button? */
  std::shared_ptr<Preview> preview; /**< Current solution trajectory from the walking pattern generator */
  std::shared_ptr<mc_tasks::OrientationTask> pelvisTask; /**< Pelvis orientation task */
  std::shared_ptr<mc_tasks::OrientationTask> torsoTask; /**< Torso orientation task */
  std::vector<std::vector<double>>
      halfSitPose; /**< Half-sit joint-angle configuration stored when the controller starts. */

private: /* hidden from FSM states */
  Eigen::Matrix3d pelvisOrientation_ = Eigen::Matrix3d::Identity(); /**< Pelvis orientation (upright by default) */
  Eigen::Vector3d realCom_ = Eigen::Vector3d::Zero(); /**< Estimated CoM position of the robot */
  Eigen::Vector3d realComd_ = Eigen::Vector3d::Zero(); /**< Estimated CoM velocity of the robot */
  FloatingBaseObserver floatingBaseObs_; /**< Floating base observer */
  LowPassVelocityFilter<Eigen::Vector3d> comVelFilter_; /**< Low-pass filter used to estimate the CoM velocity */
  ModelPredictiveControl mpc_; /**< MPC problem solver used for walking pattern generation */
  NetWrenchObserver netWrenchObs_; /**< Observe the net contact wrench exerted on the robot */
  Pendulum pendulum_; /**< Holds the reference state (CoM position, velocity, ZMP, ...) from the walking pattern */
  Sole sole_; /**< Sole dimensions of the robot model */
  Stabilizer stabilizer_; /**< Balance feedback control component that updates task targets */
  bool leftFootRatioJumped_ = false; /**< Flag used to avoid discontinuous CoM velocity updates */
  double ctlTime_ = 0.; /**< Controller time */
  double defaultTorsoPitch_ = 0.; /**< Default torso pitch angle, in [rad] */
  double doubleSupportDurationOverride_ = -1.; // [s]
  double leftFootRatio_ = 0.5; /**< Weight distribution ratio (0: all weight on right foot, 1: all on left foot) */
  double maxCoMHeight_ = 2.; /**< Maximum height for the center of mass (updated from configuration) */
  double maxTorsoPitch_ = -0.2; /**< Joint angle upper bound for torso pitch joint, in [rad] */
  double minCoMHeight_ = 0.; /**< Minimum height for the center of mass (updated from configuration) */
  double minTorsoPitch_ = -0.2; /**< Joint angle lower bound for torso pitch joint, in [rad] */
  double torsoPitch_; /**< Torso pitch angle with respect to the pelvis body, in [rad] */
  mc_rtc::Configuration mpcConfig_; /**< Configuration dictionary for the walking pattern generator */
  std::string segmentName_ = ""; /**< Name of current log segment (this is an mc_rtc specific) */
  unsigned nbLogSegments_ = 100; /**< Index used to number log segments (this is an mc_rtc specific) */
  unsigned nbMPCFailures_ = 0; /**< Number of times the walking pattern generator failed */
};

} // namespace lipm_walking
