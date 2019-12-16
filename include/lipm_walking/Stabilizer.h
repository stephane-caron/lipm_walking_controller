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

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>

#include <eigen-quadprog/QuadProg.h>
#include <lipm_walking/Contact.h>
#include <lipm_walking/Pendulum.h>
#include <lipm_walking/Sole.h>
#include <lipm_walking/utils/ExponentialMovingAverage.h>
#include <lipm_walking/utils/LeakyIntegrator.h>
#include <lipm_walking/utils/StationaryOffsetFilter.h>

namespace lipm_walking
{

/** Walking stabilization based on linear inverted pendulum tracking.
 *
 * Stabilization bridges the gap between the open-loop behavior of the
 * pendulum state reference (feedforward controls) and feedback read from
 * state estimation. In our case, feedback is done on the DCM of the LIPM:
 *
 * \f[
 *   \dot{\xi} = \dot{\xi}^{d} + k_p (\xi^d - \xi) + k_i \int (\xi^d - \xi)
 * \f]
 *
 * Which boils down into corresponding formulas for the CoP and CoM
 * acceleration targets.
 *
 */
struct Stabilizer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr double MAX_AVERAGE_DCM_ERROR = 0.05; /**< Maximum average (integral) DCM error in [m] */
  static constexpr double MAX_COM_ADMITTANCE = 20; /**< Maximum admittance for CoM admittance control */
  static constexpr double MAX_COP_ADMITTANCE = 0.1; /**< Maximum CoP admittance for foot damping control */
  static constexpr double MAX_DCM_D_GAIN = 2.; /**< Maximum DCM derivative gain (no unit) */
  static constexpr double MAX_DCM_I_GAIN = 100.; /**< Maximum DCM average integral gain in [Hz] */
  static constexpr double MAX_DCM_P_GAIN = 20.; /**< Maximum DCM proportional gain in [Hz] */
  static constexpr double MAX_DFZ_ADMITTANCE =
      5e-4; /**< Maximum admittance in [s] / [kg] for foot force difference control */
  static constexpr double MAX_DFZ_DAMPING =
      10.; /**< Maximum normalized damping in [Hz] for foot force difference control */
  static constexpr double MAX_FDC_RX_VEL =
      0.2; /**< Maximum x-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_FDC_RY_VEL =
      0.2; /**< Maximum y-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_FDC_RZ_VEL =
      0.2; /**< Maximum z-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_ZMPCC_COM_OFFSET = 0.05; /**< Maximum CoM offset due to admittance control in [m] */
  static constexpr double MIN_DSP_FZ = 15.; /**< Minimum normal contact force in [N] for DSP, used to avoid low-force
                                               targets when close to contact switches. */

  /** Initialize stabilizer.
   *
   * \param robot Robot model.
   *
   * \param pendulum CoM state reference placeholder.
   *
   * \param dt Controller timestep.
   *
   */
  Stabilizer(const mc_rbdyn::Robot & robot, const Pendulum & pendulum, double dt);

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
  bool detectTouchdown(const std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact);

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
   * This function has no effect when the measured normal force is already
   * higher than the target. Otherwise, it will set a positive admittance
   * along the z-axis of the contact frame.
   *
   */
  void seekTouchdown(std::shared_ptr<mc_tasks::force::CoPTask> footTask);

  /** Configure foot task for contact at a given location.
   *
   * \param footTask One of leftFootTask or rightFootTask.
   *
   * \param contact Target contact location.
   *
   */
  void setContact(std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact);

  /** Configure foot task for swinging.
   *
   * \param footTask One of leftFootTask or rightFootTask.
   *
   * Foot target is reset to the current frame pose.
   *
   */
  void setSwingFoot(std::shared_ptr<mc_tasks::force::CoPTask> footTask);

  /** Get contact state.
   *
   * \returns contactState Contact state.
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

  /** Set model sole properties.
   *
   * \param sole Sole parameters.
   *
   */
  void sole(const Sole & sole)
  {
    sole_ = sole;
  }

  /** Update real-robot state.
   *
   * \param com Position of the center of mass.
   *
   * \param comd Velocity of the center of mass.
   *
   * \param wrench Net contact wrench in the inertial frame.
   *
   * \param leftFootRatio Desired force distribution ratio for left foot.
   *
   */
  void updateState(const Eigen::Vector3d & com,
                   const Eigen::Vector3d & comd,
                   const sva::ForceVecd & wrench,
                   double leftFootRatio);

  /** Update H-representation of contact wrench cones.
   *
   * \param sole Sole parameters.
   *
   * See <https://hal.archives-ouvertes.fr/hal-02108449/document> for
   * technical details on the derivation of this formula.
   *
   */
  void wrenchFaceMatrix(const Sole & sole)
  {
    double X = sole.halfLength;
    double Y = sole.halfWidth;
    double mu = sole.friction;
    // clang-format off
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
    // clang-format on
  }

  /** ZMP target after wrench distribution.
   *
   * \returns zmp ZMP target in the inertial frame.
   *
   */
  Eigen::Vector3d zmp() const
  {
    return computeZMP(distribWrench_);
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
      double forceRatioWeight = config("force_ratio");
      double netWrenchWeight = config("net_wrench");
      ankleTorqueSqrt = std::sqrt(ankleTorqueWeight);
      forceRatioSqrt = std::sqrt(forceRatioWeight);
      netWrenchSqrt = std::sqrt(netWrenchWeight);
    }

  public:
    double ankleTorqueSqrt;
    double forceRatioSqrt;
    double netWrenchSqrt;
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
  void distributeWrenchQuadProg(const sva::ForceVecd & desiredWrench);

  /** Project desired wrench to single support foot.
   *
   * \param desiredWrench Desired resultant reaction wrench.
   *
   * \param footTask Target foot.
   *
   */
  void saturateWrench(const sva::ForceVecd & desiredWrench, std::shared_ptr<mc_tasks::force::CoPTask> & footTask);
  void saturateWrenchQuadProg(const sva::ForceVecd & desiredWrench,
                              std::shared_ptr<mc_tasks::force::CoPTask> & footTask);

  /** Reset admittance, damping and stiffness for every foot in contact.
   *
   */
  void setSupportFootGains();

  /** Update CoM task with ZMP Compensation Control.
   *
   * This approach is based on Section 6.2.2 of Dr Nagasaka's PhD thesis
   * "体幹位置コンプライアンス制御によるモデル誤差吸収" (1999) from
   * <https://sites.google.com/site/humanoidchannel/home/publication>.
   * The main differences is that the CoM offset is (1) implemented as CoM
   * damping control with an internal leaky integrator and (2) computed from
   * the distributed rather than reference ZMP.
   *
   */
  void updateCoMTaskZMPCC();

  /** Apply foot force difference control.
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
   * \returns contactAdmittance Admittance of contact task.
   *
   */
  sva::ForceVecd contactAdmittance()
  {
    return {{copAdmittance_.y(), copAdmittance_.x(), 0.}, {0., 0., 0.}};
  }

public:
  Contact leftFootContact; /**< Current left foot contact */
  Contact rightFootContact; /**< Current right foot contact */
  std::shared_ptr<mc_tasks::CoMTask> comTask; /**< CoM position task */
  std::shared_ptr<mc_tasks::force::CoPTask> leftFootTask; /**< Left foot hybrid position/force control task */
  std::shared_ptr<mc_tasks::force::CoPTask> rightFootTask; /**< Right foot hybrid position/force control task */

private:
  ContactState contactState_ = ContactState::DoubleSupport; /**< Desired contact state */
  Eigen::Matrix<double, 16, 6> wrenchFaceMatrix_; /**< Matrix of single-contact wrench cone inequalities */
  Eigen::QuadProgDense qpSolver_; /**< Least-squares solver for wrench distribution */
  Eigen::Vector2d comAdmittance_ = Eigen::Vector2d::Zero(); /**< Admittance gains for CoM admittance control */
  Eigen::Vector2d copAdmittance_ = Eigen::Vector2d::Zero(); /**< Admittance gains for foot damping control */
  Eigen::Vector3d comStiffness_ = {1000., 1000., 100.}; /**< Proportional gain of CoM IK task */
  Eigen::Vector3d dcmAverageError_ =
      Eigen::Vector3d::Zero(); /**< DCM average error used to compute ZMP integral feedback */
  Eigen::Vector3d dcmError_ = Eigen::Vector3d::Zero(); /**< DCM error used to compute ZMP proportional feedback */
  Eigen::Vector3d dcmVelError_ =
      Eigen::Vector3d::Zero(); /**< DCM derivative error used to compute ZMP derivative feedback */
  Eigen::Vector3d measuredCoM_ = Eigen::Vector3d::Zero(); /**< Estimated CoM position */
  Eigen::Vector3d measuredCoMd_ = Eigen::Vector3d::Zero(); /**< Estimated CoM velocity */
  Eigen::Vector3d measuredZMP_ = Eigen::Vector3d::Zero(); /**< Estimated ZMP position */
  Eigen::Vector3d zmpError_ = Eigen::Vector3d::Zero(); /**< Error between reference and measured ZMP */
  Eigen::Vector3d zmpccCoMAccel_ = Eigen::Vector3d::Zero(); /**< Additional CoM acceleration from ZMPCC */
  Eigen::Vector3d zmpccCoMOffset_ = Eigen::Vector3d::Zero(); /**< Additional CoM position offset from ZMPCC */
  Eigen::Vector3d zmpccCoMVel_ = Eigen::Vector3d::Zero(); /**< Additional CoM velocity from ZMPCC */
  Eigen::Vector3d zmpccError_ = Eigen::Vector3d::Zero(); /**< Error between distributed and measured ZMP */
  Eigen::Vector4d polePlacement_ = {-10., -5., -1., 10.}; /**< Pole placement with ZMP delay (Morisawa et al., 2014) */
  ExponentialMovingAverage dcmIntegrator_; /**< Moving average (a.k.a. leaky integrator) of the DCM error */
  FDQPWeights fdqpWeights_; /**< Weights of the wrench distribution quadratic program */
  LeakyIntegrator zmpccIntegrator_; /**< Leaky integrator for the CoM offset added by ZMPCC */
  StationaryOffsetFilter dcmDerivator_; /**< Filter used to evaluate the derivative of the DCM error */
  bool inTheAir_ = false; /**< Is the robot in the air? */
  bool zmpccOnlyDS_ = true; /**< Apply CoM admittance control only in double support? */
  const Pendulum & pendulum_; /**< Reference to desired reduced-model state */
  const mc_rbdyn::Robot & controlRobot_; /**< Control robot model (input to joint position controllers) */
  double comWeight_ = 1000.; /**< Weight of CoM IK task */
  double contactWeight_ = 100000.; /**< Weight of contact IK tasks */
  double dcmDerivGain_ = 0.; /**< Derivative gain on DCM error */
  double dcmIntegralGain_ = 5.; /**< Integral gain on DCM error */
  double dcmPropGain_ = 1.; /**< Proportional gain on DCM error */
  double dfzAdmittance_ = 1e-4; /**< Admittance for foot force difference control */
  double dfzDamping_ = 0.; /**< Damping term in foot force difference control */
  double dfzForceError_ = 0.; /**< Force error in foot force difference control */
  double dfzHeightError_ = 0.; /**< Height error in foot force difference control */
  double dt_ = 0.005; /**< Controller cycle in [s] */
  double leftFootRatio_ = 0.5; /**< Weight distribution ratio (0: all weight on right foot, 1: all on left foot) */
  double mass_ = 38.; /**< Robot mass in [kg] */
  double runTime_ = 0.; /**< Performance of the run() function in [ms] */
  double swingFootStiffness_ = 2000.; /**< Stiffness of swing foot IK task */
  double swingFootWeight_ = 500.; /**< Weight of swing foot IK task */
  double vdcFrequency_ = 1.; /**< Frequency used in double-support vertical drift compensation */
  double vdcHeightError_ = 0.; /**< Average height error used in vertical drift compensation */
  double vdcStiffness_ = 1000.; /**< Stiffness used in single-support vertical drift compensation */
  Sole sole_; /**< Sole dimensions of the robot model */
  mc_rtc::Configuration config_; /**< Stabilizer configuration dictionary */
  std::vector<std::string> comActiveJoints_; /**< Joints used by CoM IK task */
  sva::ForceVecd distribWrench_ = sva::ForceVecd::Zero(); /**< Target net contact wrench after wrench distribution */
  sva::ForceVecd measuredWrench_ = sva::ForceVecd::Zero(); /**< Net contact wrench measured from sensors */
  sva::MotionVecd contactDamping_ = sva::MotionVecd::Zero(); /**< Derivative gain of contact IK task */
  sva::MotionVecd contactStiffness_ = sva::MotionVecd::Zero(); /**< Proportional gain of contact IK task */
  sva::PTransformd zmpFrame_ = sva::PTransformd::Identity(); /**< Frame in which the ZMP is taken */
};

} // namespace lipm_walking
