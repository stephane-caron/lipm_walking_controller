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

#include <eigen-lssol/LSSOL_LS.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>

#include <lipm_walking/Pendulum.h>
#include <lipm_walking/Contact.h>
#include <lipm_walking/Sole.h>
#include <lipm_walking/defs.h>
#include <lipm_walking/utils/LeakyIntegrator.h>
#include <lipm_walking/utils/rotations.h>
#include <lipm_walking/utils/stats.h>

namespace lipm_walking
{
  /** Walking stabilization based on linear inverted pendulum tracking.
   *
   * Stabilization bridges the gap between the open-loop behavior of the
   * pendulum state reference (feedforward controls) and feedback read from
   * state estimation. In our case, feedback is done on the DCM of the LIPM:
   *
   *    \dot{\xi} = \dot{\xi}^{d} + k_p (\xi^d - \xi) + k_i \int (\xi^d - \xi)
   *
   * Which boils down into corresponding formulas for the CoP and CoM
   * acceleration targets.
   *
   */
  struct Stabilizer
  {
    /* Maximum angular velocities for foot damping control. */
    static constexpr double MAX_FDC_RX_VEL = 0.2; // [rad] / [s]
    static constexpr double MAX_FDC_RY_VEL = 0.2; // [rad] / [s]
    static constexpr double MAX_FDC_RZ_VEL = 0.2; // [rad] / [s]

    /* Maximum gains for HRP4LIRMM in standing static equilibrium. */
    static constexpr double MAX_COM_ADMITTANCE = 20;
    static constexpr double MAX_COP_ADMITTANCE = 0.05;
    static constexpr double MAX_DCM_I_GAIN = 20.;
    static constexpr double MAX_DCM_P_GAIN = 10.;
    static constexpr double MAX_DFZ_ADMITTANCE = 1.75e-4;
    static constexpr double MAX_ZMP_GAIN = 20.;

    /* Avoid low-pressure targets too close to contact switches */
    static constexpr double MIN_DS_PRESSURE = 15.; // [N]

    /* Saturate integrator in case of windup */
    static constexpr double MAX_AVERAGE_DCM_ERROR = 0.05; // [m]
    static constexpr double MAX_ZMPCC_COM_OFFSET = 0.05; // [m]

    /** Initialize stabilizer.
     *
     * \param robot Robot model.
     *
     * \param ref CoM state reference placeholder.
     *
     * \param dt Controller timestep.
     *
     */
    Stabilizer(const mc_rbdyn::Robot & robot, const Pendulum & ref, double dt);

    /** Add GUI panel.
     *
     * \param gui GUI handle.
     *
     */
    void addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);

    /** Log stabilizer entries.
     *
     * \param logger Logger.
     *
     */
    void addLogEntries(mc_rtc::Logger & logger);

    /** Add tasks to QP solver.
     *
     * \param solver QP solver to add tasks to.
     *
     */
    void addTasks(mc_solver::QPSolver & solver);

    /** Disable all feedback components.
     *
     */
    void disable();

    /** Compute ZMP of a wrench in the output frame.
     *
     * \param wrench Wrench at the origin of the world frame.
     *
     */
    Eigen::Vector3d computeZMP(const sva::ForceVecd & wrench) const;

    /** Read configuration from dictionary.
     *
     */
    void configure(const mc_rtc::Configuration &);

    /** Detect foot touchdown based on both force and distance.
     *
     * \param footTask Swing foot task.
     *
     * \param contact Target contact.
     *
     */
    bool detectTouchdown(const std::shared_ptr<mc_tasks::CoPTask> footTask, const Contact & contact);

    /** Apply stored configuration.
     *
     */
    void reconfigure();

    /** Remove tasks from QP solver.
     *
     * \param solver QP solver to remove tasks from.
     *
     */
    void removeTasks(mc_solver::QPSolver & solver);

    /** Reset CoM and foot CoP tasks.
     *
     * \param robots Robots where the task will be applied.
     *
     */
    void reset(const mc_rbdyn::Robots & robots);

    /** Update QP task targets.
     *
     * This function is called once the reference has been updated.
     *
     */
    void run();

    /** Configure foot task for contact seeking.
     *
     * \param footTask One of leftFootTask or rightFootTask.
     *
     * This function has no effect when the measured pressure is already higher
     * than the target. Otherwise, it will set a positive admittance along the
     * z-axis of the contact frame.
     *
     */
    void seekTouchdown(std::shared_ptr<mc_tasks::CoPTask> footTask);

    /** Configure foot task for contact at a given location.
     *
     * \param footTask One of leftFootTask or rightFootTask.
     * 
     * \param contact Target contact location.
     *
     */
    void setContact(std::shared_ptr<mc_tasks::CoPTask> footTask, const Contact & contact);

    /** Configure foot task for swinging.
     *
     * \param footTask One of leftFootTask or rightFootTask.
     *
     * Foot target is reset to the current frame pose.
     *
     */
    void setSwingFoot(std::shared_ptr<mc_tasks::CoPTask> footTask);

    /** Get contact state.
     *
     */
    ContactState contactState()
    {
      return contactState_;
    }

    /** Set desired contact state.
     *
     */
    void contactState(ContactState contactState)
    {
      contactState_ = contactState;
    }

    /** Update robot mass.
     *
     */
    void mass(double mass)
    {
      mass_ = mass;
    }

    /** Update real-robot state.
     *
     * \param com Position of the center of mass.
     *
     * \param comd Velocity of the center of mass.
     *
     * \param wrench Net contact wrench in the inertial frame.
     *
     * \param leftFootRatio Desired pressure distribution ratio for left foot.
     *
     */
    void updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd, const sva::ForceVecd & wrench, double leftFootRatio);

    /** Update H-representation of contact wrench cones.
     *
     * \param sole Sole parameters.
     *
     * This formula is derived in "Stability of Surface Contacts for Humanoid
     * Robots: Closed-Form Formulae of the Contact Wrench Cone for Rectangular
     * Support Areas" (Caron et al., ICRA 2015).
     *
     */
    void wrenchFaceMatrix(const Sole & sole)
    {
      double X = sole.halfLength;
      double Y = sole.halfWidth;
      double mu = sole.friction;
      wrenchFaceMatrix_ <<
        // mx,  my,  mz,  fx,  fy,            fz,
            0,   0,   0,  -1,   0,           -mu,
            0,   0,   0,  +1,   0,           -mu,
            0,   0,   0,   0,  -1,           -mu,
            0,   0,   0,   0,  +1,           -mu,
           -1,   0,   0,   0,   0,            -Y,
           +1,   0,   0,   0,   0,            -Y,
            0,  -1,   0,   0,   0,            -X,
            0,  +1,   0,   0,   0,            -X,
          +mu, +mu,  -1,  -Y,  -X, -(X + Y) * mu,
          +mu, -mu,  -1,  -Y,  +X, -(X + Y) * mu,
          -mu, +mu,  -1,  +Y,  -X, -(X + Y) * mu,
          -mu, -mu,  -1,  +Y,  +X, -(X + Y) * mu,
          +mu, +mu,  +1,  +Y,  +X, -(X + Y) * mu,
          +mu, -mu,  +1,  +Y,  -X, -(X + Y) * mu,
          -mu, +mu,  +1,  -Y,  +X, -(X + Y) * mu,
          -mu, -mu,  +1,  -Y,  -X, -(X + Y) * mu;
    }

    /** ZMP target after force distribution.
     *
     */
    Eigen::Vector3d zmp() const
    {
      return computeZMP(distribWrench_);
    }

    /** Difference between desired and measured ZMP.
     *
     */
    const Eigen::Vector3d & zmpError()
    {
      return zmpError_;
    }

  private:
    /** Weights for force distribution quadratic program (FDQP).
     *
     */
    struct FDQPWeights
    {
      /** Read force distribution QP weights from configuration.
       *
       * \param config Configuration dictionary.
       *
       */
      void configure(const mc_rtc::Configuration & config)
      {
        double ankleTorqueWeight = config("ankle_torque");
        double netWrenchWeight = config("net_wrench");
        double pressureWeight = config("pressure");
        ankleTorqueSqrt = std::sqrt(ankleTorqueWeight);
        netWrenchSqrt = std::sqrt(netWrenchWeight);
        pressureSqrt = std::sqrt(pressureWeight);
      }

    public:
      double ankleTorqueSqrt;
      double netWrenchSqrt;
      double pressureSqrt;
    };

    /** Check that all gains are within boundaries.
     *
     */
    void checkGains();

    /** Check whether the robot is in the air.
     *
     */
    void checkInTheAir();

    /** Compute desired wrench based on DCM error.
     *
     */
    sva::ForceVecd computeDesiredWrench();

    /** Distribute a desired wrench in double support.
     *
     * \param desiredWrench Desired resultant reaction wrench.
     *
     */
    void distributeWrench(const sva::ForceVecd & desiredWrench);

    /** Project desired wrench to single support foot.
     *
     * \param desiredWrench Desired resultant reaction wrench.
     *
     * \param footTask Target foot.
     *
     */
    void saturateWrench(const sva::ForceVecd & desiredWrench, std::shared_ptr<mc_tasks::CoPTask> & footTask);

    /** Reset admittance, damping and stiffness for every foot in contact.
     *
     */
    void setSupportFootGains();

    /** Simplest CoM control law: no feedback.
     *
     */
    //void updateCoMOpenLoop();

    /** Send desired net force after QP distribution.
     *
     * This approach is e.g. found in "Walking on Partial Footholds Including
     * Line Contacts with the Humanoid Robot Atlas" (Wiedebach et al.,
     * Humanoids 2016).
     *
     */
    //void updateCoMDistribForce();

    /** CoM compliance control.
     *
     * This approach is e.g. found in "Stabilization for the compliant humanoid
     * robot COMAN exploiting intrinsic and controlled compliance" (Li et al.,
     * ICRA 2012).
     *
     */
    //void updateCoMComplianceControl();

    /** CoM acceleration used for additional reaction force tracking.
     *
     * Same idea as ZMPCC but replacing the ZMP with the reaction force.
     * Problem with looking at force rather than torque readings is that it
     * yields steady-state ZMP error (standing upright, yes but not with the
     * ZMP at the correct location).
     *
     */
    //void updateCoMForceTracking();

    /** ZMP Compensation Control.
     *
     * This implementation is based on
     * "体幹位置コンプライアンス制御によるモデル誤差吸収" (Section 6.2.2 of
     * Nagasaka's PhD thesis, 1999), available from
     * <http://www.geocities.jp/kamono3/public.htm>.
     *
     */
    //void updateCoMPosZMPCC();

    /** ZMP Compensation Control applied after ZMP distribution.
     *
     * Same approach as Nagasaka's ZMPCC but (1) implemented as CoM
     * acceleration reference sent to the inverse kinematics and (2) applied to
     * the distributed ZMP.
     *
     */
    //void updateCoMAccelZMPCC();

    /** Update CoM task with ZMP Compensation Control.
     *
     * Same approach as Nagasaka's ZMPCC but (1) implemented as CoM damping
     * control with an internal leaky integrator and (2) applied to the
     * distributed ZMP.
     *
     */
    void updateCoMZMPCC();

    /** Apply foot pressure difference control.
     *
     * This method is described in Section III.E of "Biped walking
     * stabilization based on linear inverted pendulum tracking" (Kajita et
     * al., IROS 2010).
     *
     */
    void updateFootForceDifferenceControl();

    /** Update ZMP frame from contact state.
     *
     */
    void updateZMPFrame();

    /** Get 6D contact admittance vector from 2D CoP admittance.
     *
     */
    sva::ForceVecd contactAdmittance()
    {
      return {{copAdmittance_.y(), copAdmittance_.x(), 0.}, {0., 0., 0.}};
    }

  public:
    Contact leftFootContact;
    Contact rightFootContact;
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::CoPTask> leftFootTask;
    std::shared_ptr<mc_tasks::CoPTask> rightFootTask;

  private:
    ContactState contactState_ = ContactState::DoubleSupport;
    Eigen::LSSOL_LS wrenchSolver_;
    Eigen::Matrix<double, 16, 6> wrenchFaceMatrix_;
    Eigen::Vector2d comAdmittance_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d copAdmittance_ = Eigen::Vector2d::Zero();
    Eigen::Vector3d comStiffness_ = {1000., 1000., 100.};
    Eigen::Vector3d dcmAverageError_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d dcmError_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d desiredCoMAccel_;
    Eigen::Vector3d measuredCoM_;
    Eigen::Vector3d measuredCoMd_;
    Eigen::Vector3d measuredZMP_;
    Eigen::Vector3d zmpError_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d zmpccCoMAccel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d zmpccCoMOffset_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d zmpccCoMVel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d zmpccError_ = Eigen::Vector3d::Zero();
    ExponentialMovingAverage dcmIntegrator_;
    FDQPWeights fdqpWeights_;
    LeakyIntegrator zmpccIntegrator_;
    bool inTheAir_ = false;
    bool zmpccOnlyDS_ = true;
    const Pendulum & pendulum_;
    const mc_rbdyn::Robot & controlRobot_;
    double comWeight_ = 1000.;
    double contactWeight_ = 100000.;
    double dcmGain_ = 1.; /**< Proportional gain on DCM error */
    double dcmIntegralGain_ = 5.; /**< Integral gain on DCM error */
    double dfzAdmittance_ = 1e-4;
    double dt_ = 0.005; // [s]
    double leftFootRatio_ = 0.5;
    double logMeasuredDFz_ = 0.;
    double logMeasuredSTz_ = 0.;
    double logTargetDFz_ = 0.;
    double logTargetSTz_ = 0.;
    double mass_ = 38.; // [kg]
    double runTime_ = 0.;
    double swingFootStiffness_ = 2000.;
    double swingFootWeight_ = 500.;
    double vdcDamping_ = 0.; /**< Vertical Drift Compensation damping */
    double vdcFrequency_ = 1.; /**< Vertical Drift Compensation frequency */
    double vdcStiffness_ = 1000.; /**< Vertical Drift Compensation stiffness */
    double vdcZPos_ = 0.;
    double vfcZCtrl_ = 0.;
    double zmpGain_ = 1.; /**< Gain on ZMP error */
    mc_rtc::Configuration config_;
    std::vector<std::string> comActiveJoints_;
    sva::ForceVecd distribWrench_ = sva::ForceVecd::Zero();
    sva::ForceVecd measuredWrench_;
    sva::MotionVecd contactDamping_;
    sva::MotionVecd contactStiffness_;
    sva::PTransformd zmpFrame_;
  };
}
