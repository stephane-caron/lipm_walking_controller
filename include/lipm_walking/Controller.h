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

#pragma once

#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <lipm_walking/Contact.h>
#include <lipm_walking/FloatingBaseObserver.h>
#include <lipm_walking/FootstepPlan.h>
#include <lipm_walking/ModelPredictiveControl.h>
#include <lipm_walking/NetWrenchObserver.h>
#include <lipm_walking/Pendulum.h>
#include <lipm_walking/Sole.h>
#include <lipm_walking/Stabilizer.h>
#include <lipm_walking/defs.h>
#include <lipm_walking/utils/LowPassVelocityFilter.h>
#include <lipm_walking/utils/clamp.h>
#include <lipm_walking/utils/rotations.h>

namespace lipm_walking
{
  /** Preview update period, same as MPC sampling period.
   *
   */
  constexpr double PREVIEW_UPDATE_PERIOD = ModelPredictiveControl::SAMPLING_PERIOD;

  // The following constants depend on the robot model (here HRP-4)
  constexpr double MAX_CHEST_P = +0.4; // [rad], DOF limit is +0.5 [rad]
  constexpr double MIN_CHEST_P = -0.1; // [rad], DOF limit is -0.2 [rad]

  /** Walking controller.
   *
   */
  struct MC_CONTROL_DLLAPI Controller : public mc_control::fsm::Controller
  {
    /** Initialization of the controller. 
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

    /** List available contact plans.
     *
     */
    std::vector<std::string> availablePlans() const
    {
      return plans_.keys();
    }

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
      if (doubleSupportDurationOverride_ > 0.)
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
    FootstepPlan plan;
    bool emergencyStop = false;
    bool pauseWalking = false;
    bool pauseWalkingRequested = false;
    std::shared_ptr<Preview> preview;
    std::shared_ptr<mc_tasks::OrientationTask> pelvisTask;
    std::shared_ptr<mc_tasks::OrientationTask> torsoTask;
    std::vector<std::vector<double>> halfSitPose;

  private: /* hidden from FSM states */
    Eigen::Matrix3d pelvisOrientation_ = Eigen::Matrix3d::Identity(); // keep pelvis upright
    Eigen::Vector3d controlCom_;
    Eigen::Vector3d controlComd_;
    Eigen::Vector3d realCom_;
    Eigen::Vector3d realComd_;
    FloatingBaseObserver floatingBaseObs_;
    LowPassVelocityFilter<Eigen::Vector3d> comVelFilter_;
    ModelPredictiveControl mpc_;
    NetWrenchObserver netWrenchObs_;
    Pendulum pendulum_;
    Sole sole_;
    Stabilizer stabilizer_;
    bool leftFootRatioJumped_ = false;
    double ctlTime_ = 0.;
    double defaultTorsoPitch_ = 0.1; // [rad]
    double doubleSupportDurationOverride_ = -1.; // [s]
    double leftFootRatio_ = 0.5;
    double torsoPitch_;
    mc_rtc::Configuration mpcConfig_;
    mc_rtc::Configuration plans_;
    std::string segmentName_ = "";
    unsigned nbLogSegments_ = 100;
    unsigned nbMPCFailures_ = 0;
  };
}
