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

#include <chrono>

#include <lipm_walking/Stabilizer.h>
#include <lipm_walking/utils/clamp.h>

namespace lipm_walking
{
  namespace
  {
    inline Eigen::Vector2d vecFromError(const Eigen::Vector3d & error)
    {
      double x = -std::round(error.x() * 1000.);
      double y = -std::round(error.y() * 1000.);
      return Eigen::Vector2d{x, y};
    }
  }

  Stabilizer::Stabilizer(const mc_rbdyn::Robot & controlRobot, const Pendulum & pendulum, double dt)
    : dcmIntegrator_(dt, /* timeConstant = */ 5.),
      pendulum_(pendulum),
      controlRobot_(controlRobot),
      dt_(dt),
      mass_(controlRobot.mass())
  {
  }

  void Stabilizer::addLogEntries(mc_rtc::Logger & logger)
  {
    logger.addLogEntry("stabilizer_contact_state",
      [this]() -> double
      {
        switch (contactState_)
        {
          case ContactState::DoubleSupport:
            return 0;
          case ContactState::LeftFoot:
            return 1;
          case ContactState::RightFoot:
            return -1;
          default:
            return -3;
        }
      });
    logger.addLogEntry("error_dcm", [this]() { return dcmError_; });
    logger.addLogEntry("error_dcmAverage", [this]() { return dcmAverageError_; });
    logger.addLogEntry("error_dfz", [this]() { return logTargetDFz_ - logMeasuredDFz_; });
    logger.addLogEntry("error_sfz", [this]() { return logTargetSTz_ - logMeasuredSTz_; });
    logger.addLogEntry("error_zmp", [this]() { return zmpError_; });
    logger.addLogEntry("perf_Stabilizer", [this]() { return runTime_; });
    logger.addLogEntry("stabilizer_admittance_com", [this]() { return comAdmittance_; });
    logger.addLogEntry("stabilizer_admittance_cop", [this]() { return copAdmittance_; });
    logger.addLogEntry("stabilizer_admittance_dfz", [this]() { return dfzAdmittance_; });
    logger.addLogEntry("stabilizer_fdqp_weights_ankleTorque", [this]() { return std::pow(fdqpWeights_.ankleTorqueSqrt, 2); });
    logger.addLogEntry("stabilizer_fdqp_weights_netWrench", [this]() { return std::pow(fdqpWeights_.netWrenchSqrt, 2); });
    logger.addLogEntry("stabilizer_fdqp_weights_pressure", [this]() { return std::pow(fdqpWeights_.pressureSqrt, 2); });
    logger.addLogEntry("stabilizer_integrator_timeConstant", [this]() { return dcmIntegrator_.timeConstant(); });
    logger.addLogEntry("stabilizer_lipm_tracking_dcm", [this]() { return dcmGain_; });
    logger.addLogEntry("stabilizer_lipm_tracking_dcmIntegral", [this]() { return dcmIntegralGain_; });
    logger.addLogEntry("stabilizer_lipm_tracking_zmp", [this]() { return zmpGain_; });
    logger.addLogEntry("stabilizer_vdc_damping", [this]() { return vdcDamping_; });
    logger.addLogEntry("stabilizer_vdc_frequency", [this]() { return vdcFrequency_; });
    logger.addLogEntry("stabilizer_vdc_stiffness", [this]() { return vdcStiffness_; });
    logger.addLogEntry("stabilizer_vdc_z_pos", [this]() { return vdcZPos_; });
    logger.addLogEntry("stabilizer_vfc_dfz_measured", [this]() { return logMeasuredDFz_; });
    logger.addLogEntry("stabilizer_vfc_dfz_target", [this]() { return logTargetDFz_; });
    logger.addLogEntry("stabilizer_vfc_stz_measured", [this]() { return logMeasuredSTz_; });
    logger.addLogEntry("stabilizer_vfc_stz_target", [this]() { return logTargetSTz_; });
    logger.addLogEntry("stabilizer_vfc_z_ctrl", [this]() { return vfcZCtrl_; });
    logger.addLogEntry("stabilizer_zmp", [this]() { return zmp(); });
    logger.addLogEntry("stabilizer_zmpcc_comdd_offset", [this]() { return zmpccAccelOffset_; });
  }

  void Stabilizer::addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
  {
    using namespace mc_rtc::gui;
    gui->addElement(
      {"Stabilizer", "Gains"},
      Button(
        "Disable",
        [this]() { disable(); }),
      Button(
        "Reconfigure",
        [this]() { reconfigure(); }),
      ArrayInput(
        "Foot admittance",
        {"CoPx", "CoPy", "DFz"},
        [this]() -> Eigen::Vector3d
        {
          return {copAdmittance_.x(), copAdmittance_.y(), dfzAdmittance_};
        },
        [this](const Eigen::Vector3d & a)
        {
          copAdmittance_.x() = clamp(a(0), 0., MAX_COP_ADMITTANCE);
          copAdmittance_.y() = clamp(a(1), 0., MAX_COP_ADMITTANCE);
          dfzAdmittance_ = clamp(a(2), 0., MAX_DFZ_ADMITTANCE);
        }),
      ArrayInput(
        "LIPM tracking",
        {"DCMp", "DCMi", "ZMP"},
        [this]() -> Eigen::Vector3d { return {dcmGain_, dcmIntegralGain_, zmpGain_}; },
        [this](const Eigen::Vector3d & gains)
        {
          dcmGain_ = clamp(gains(0), 0., MAX_DCM_P_GAIN);
          dcmIntegralGain_ = clamp(gains(1), 0., MAX_DCM_I_GAIN);
          zmpGain_ = clamp(gains(2), 0., MAX_ZMP_GAIN);
        }),
      ArrayInput(
        "Vertical drift control",
        {"frequency", "stiffness", "damping"},
        [this]() -> Eigen::Vector3d { return {vdcFrequency_, vdcStiffness_, vdcDamping_}; },
        [this](const Eigen::Vector3d & v)
        {
          vdcFrequency_ = clamp(v(0), 0., 10.);
          vdcStiffness_ = clamp(v(1), 0., 1e4);
          vdcDamping_ = clamp(v(2), 0., 100.);
        }),
      ArrayInput(
        "CoM admittance",
        {"Ax", "Ay"},
        [this]() { return comAdmittance_; },
        [this](const Eigen::Vector2d & a)
        {
          comAdmittance_.x() = clamp(a.x(), 0., MAX_COM_ADMITTANCE);
          comAdmittance_.y() = clamp(a.y(), 0., MAX_COM_ADMITTANCE);
        }));
    gui->addElement(
      {"Stabilizer", "Integrator"},
      Button(
        "Reset DCM integrator",
        [this]() { dcmIntegrator_.setZero(); }),
      NumberInput(
        "DCM integrator T",
        [this]() { return dcmIntegrator_.timeConstant(); },
        [this](double T) { dcmIntegrator_.timeConstant(T); }));
    gui->addElement(
      {"Stabilizer", "Status"},
      Label(
        "Contact state",
        [this]()
        {
          switch (contactState_)
          {
            case ContactState::DoubleSupport:
              return "DoubleSupport";
            case ContactState::LeftFoot:
              return "LeftFoot";
            case ContactState::RightFoot:
            default:
              return "RightFoot";
          }
        }),
      ArrayLabel("DCM error [mm]",
        {"x", "y"},
        [this]() { return vecFromError(dcmError_); }),
      ArrayLabel("DCM avg. error [mm]",
        {"x", "y"},
        [this]() { return vecFromError(dcmAverageError_); }),
      ArrayLabel("ZMP error [mm]",
        {"x", "y"},
        [this]() { return vecFromError(zmpError_); }),
      Label("Foot height diff [mm]",
        [this]() { return std::round(1000. * vfcZCtrl_); }));
    gui->addElement(
      {"Computation times"},
      Label(
        "Stabilizer [ms]",
        [this]() { return runTimes_.str(2, false); }));
  }

  void Stabilizer::disable()
  {
    comAdmittance_.setZero();
    copAdmittance_.setZero();
    dcmGain_ = 0.;
    dcmIntegralGain_ = 0.;
    dfzAdmittance_ = 0.;
    vdcFrequency_ = 0.;
    vdcStiffness_ = 0.;
    zmpGain_ = 0.;
  }

  void Stabilizer::configure(const mc_rtc::Configuration & config)
  {
    fdqpWeights_.configure(config("fdqp_weights"));
    if (config.has("admittance"))
    {
      auto admittance = config("admittance");
      comAdmittance_ = admittance("com");
      copAdmittance_ = admittance("cop");
      dfzAdmittance_ = admittance("dfz");
    }
    if (config.has("lipm_tracking"))
    {
      auto lipm = config("lipm_tracking");
      dcmGain_ = lipm("dcm_gain");
      dcmIntegralGain_ = lipm("dcm_integral_gain");
      dcmIntegrator_.timeConstant(lipm("dcm_integrator_time_constant"));
      zmpGain_ = lipm("zmp_gain");
    }
    if (config.has("tasks"))
    {
      auto tasks = config("tasks");
      if (tasks.has("com"))
      {
        tasks("com")("active_joints", comActiveJoints_);
        tasks("com")("stiffness", comStiffness_);
        tasks("com")("weight", comWeight_);
      }
      if (tasks.has("contact"))
      {
        double d = tasks("contact")("damping");
        double k = tasks("contact")("stiffness");
        contactDamping_ = sva::MotionVecd({d, d, d}, {d, d, d});
        contactStiffness_ = sva::MotionVecd({k, k, k}, {k, k, k});
        tasks("contact")("stiffness", contactStiffness_);
        tasks("contact")("weight", contactWeight_);
      }
      if (tasks.has("swing_foot"))
      {
        tasks("swing_foot")("stiffness", swingFootStiffness_);
        tasks("swing_foot")("weight", swingFootWeight_);
      }
    }
    if (config.has("vdc"))
    {
      auto vdc = config("vdc");
      vdcDamping_ = vdc("damping");
      vdcFrequency_ = vdc("frequency");
      vdcStiffness_ = vdc("stiffness");
    }
  }

  void Stabilizer::reset(const mc_rbdyn::Robots & robots)
  {
    unsigned robotIndex = robots.robotIndex();

    comTask.reset(new mc_tasks::CoMTask(robots, robotIndex));
    comTask->selectActiveJoints(comActiveJoints_);
    comTask->setGains(comStiffness_, 2 * comStiffness_.cwiseSqrt());
    comTask->weight(comWeight_);

    leftFootTask.reset(new mc_tasks::CoPTask("LeftFootCenter", robots, robotIndex));
    rightFootTask.reset(new mc_tasks::CoPTask("RightFootCenter", robots, robotIndex));
    leftFootTask->maxAngularVel({MAX_FDC_RX_VEL, MAX_FDC_RY_VEL, MAX_FDC_RZ_VEL});
    rightFootTask->maxAngularVel({MAX_FDC_RX_VEL, MAX_FDC_RY_VEL, MAX_FDC_RZ_VEL});
    setContact(leftFootTask, leftFootTask->surfacePose());
    setContact(rightFootTask, rightFootTask->surfacePose());

    dcmIntegrator_.setZero();
    dcmIntegrator_.saturation(MAX_AVERAGE_DCM_ERROR);
  }

  void Stabilizer::checkGains()
  {
    clampInPlace(comAdmittance_.x(), 0., MAX_COM_ADMITTANCE, "CoM x-admittance");
    clampInPlace(comAdmittance_.y(), 0., MAX_COM_ADMITTANCE, "CoM y-admittance");
    clampInPlace(copAdmittance_.x(), 0., MAX_COP_ADMITTANCE, "CoP x-admittance");
    clampInPlace(copAdmittance_.y(), 0., MAX_COP_ADMITTANCE, "CoP y-admittance");
    clampInPlace(dcmGain_, 0., MAX_DCM_P_GAIN, "DCM x-gain");
    clampInPlace(dcmIntegralGain_, 0., MAX_DCM_I_GAIN, "DCM integral x-gain");
    clampInPlace(dfzAdmittance_, 0., MAX_DFZ_ADMITTANCE, "DFz admittance");
    clampInPlace(zmpGain_, 0., MAX_ZMP_GAIN, "ZMP x-gain");
  }

  void Stabilizer::addTasks(mc_solver::QPSolver & solver)
  {
    solver.addTask(comTask);
    solver.addTask(leftFootTask);
    solver.addTask(rightFootTask);
  }

  void Stabilizer::removeTasks(mc_solver::QPSolver & solver)
  {
    solver.removeTask(comTask);
    solver.removeTask(leftFootTask);
    solver.removeTask(rightFootTask);
  }

  void Stabilizer::setContact(std::shared_ptr<mc_tasks::CoPTask> footTask, const Contact & contact)
  {
    footTask->reset();
    footTask->admittance(contactAdmittance());
    footTask->setGains(contactStiffness_, contactDamping_);
    footTask->targetPose(contact.pose);
    footTask->weight(contactWeight_);
    if (footTask->surface() == "LeftFootCenter")
    {
      leftFootContact = contact;
    }
    else if (footTask->surface() == "RightFootCenter")
    {
      rightFootContact = contact;
    }
    else
    {
      LOG_ERROR("Unknown foot surface: " << footTask->surface());
    }
  }

  void Stabilizer::setSwingFoot(std::shared_ptr<mc_tasks::CoPTask> footTask)
  {
    footTask->reset();
    footTask->setCriticalGains(swingFootStiffness_);
    footTask->weight(swingFootWeight_);
  }

  bool Stabilizer::detectTouchdown(const std::shared_ptr<mc_tasks::CoPTask> footTask, const Contact & contact)
  {
    const sva::PTransformd X_0_s = footTask->surfacePose();
    const sva::PTransformd & X_0_c = contact.pose;
    sva::PTransformd X_c_s = X_0_s * X_0_c.inv();
    double xDist = std::abs(X_c_s.translation().x());
    double yDist = std::abs(X_c_s.translation().y());
    double zDist = std::abs(X_c_s.translation().z());
    double pressure = footTask->measuredWrench().force().z();
    return (xDist < 0.03 && yDist < 0.03 && zDist < 0.03 && pressure > 50.);
  }

  void Stabilizer::seekTouchdown(std::shared_ptr<mc_tasks::CoPTask> footTask)
  {
    constexpr double MAX_VEL = 0.01; // [m] / [s]
    constexpr double TOUCHDOWN_PRESSURE = 50.;  // [N]
    constexpr double DESIRED_AFZ = MAX_VEL / TOUCHDOWN_PRESSURE;
    if (footTask->measuredWrench().force().z() < TOUCHDOWN_PRESSURE)
    {
      auto a = footTask->admittance();
      double AFz = clamp(DESIRED_AFZ, 0., 1e-2, "Contact seeking admittance");
      footTask->admittance({a.couple(), {a.force().x(), a.force().y(), AFz}});
      footTask->targetForce({0., 0., TOUCHDOWN_PRESSURE});
    }
  }

  void Stabilizer::setSupportFootGains()
  {
    sva::MotionVecd vdcContactStiffness = {
      contactStiffness_.angular(),
      {vdcStiffness_, vdcStiffness_, vdcStiffness_}};
    switch (contactState_)
    {
      case ContactState::DoubleSupport:
        leftFootTask->admittance(contactAdmittance());
        leftFootTask->setGains(contactStiffness_, contactDamping_);
        rightFootTask->admittance(contactAdmittance());
        rightFootTask->setGains(contactStiffness_, contactDamping_);
        break;
      case ContactState::LeftFoot:
        leftFootTask->admittance(contactAdmittance());
        leftFootTask->setGains(vdcContactStiffness, contactDamping_);
        break;
      case ContactState::RightFoot:
        rightFootTask->admittance(contactAdmittance());
        rightFootTask->setGains(vdcContactStiffness, contactDamping_);
        break;
    }
  }

  void Stabilizer::checkInTheAir()
  {
    double LFz = leftFootTask->measuredWrench().force().z();
    double RFz = rightFootTask->measuredWrench().force().z();
    inTheAir_ = (LFz < MIN_DS_PRESSURE && RFz < MIN_DS_PRESSURE);
  }

  void Stabilizer::updateZMPFrame()
  {
    const sva::PTransformd & X_0_lc = leftFootContact.pose;
    const sva::PTransformd & X_0_rc = rightFootContact.pose;
    switch (contactState_)
    {
      case ContactState::DoubleSupport:
        zmpFrame_ = sva::interpolate(X_0_lc, X_0_rc, 0.5);
        break;
      case ContactState::LeftFoot:
        zmpFrame_ = X_0_lc;
        break;
      case ContactState::RightFoot:
        zmpFrame_ = X_0_rc;
        break;
    }
    measuredZMP_ = computeZMP(measuredWrench_);
  }

  Eigen::Vector3d Stabilizer::computeZMP(const sva::ForceVecd & wrench) const
  {
    Eigen::Vector3d n = zmpFrame_.rotation().row(2);
    Eigen::Vector3d p = zmpFrame_.translation();
    const Eigen::Vector3d & force = wrench.force();
    double pressure = n.dot(force);
    if (pressure < 1.)
    {
      double lambda = std::pow(pendulum_.omega(), 2);
      return measuredCoM_ + world::gravity / lambda; // default for logging
    }
    const Eigen::Vector3d & moment_0 = wrench.couple();
    Eigen::Vector3d moment_p = moment_0 - p.cross(force);
    return p + n.cross(moment_p) / pressure;
  }

  void Stabilizer::run()
  {
    using namespace std::chrono;
    auto startTime = high_resolution_clock::now();

    checkGains();
    checkInTheAir();
    setSupportFootGains();
    updateZMPFrame();

    auto desiredWrench = computeDesiredWrench();

    switch (contactState_)
    {
      case ContactState::DoubleSupport:
        distributeWrench(desiredWrench);
        break;
      case ContactState::LeftFoot:
        saturateWrench(desiredWrench, leftFootTask);
        rightFootTask->setZeroTargetWrench();
        break;
      case ContactState::RightFoot:
        saturateWrench(desiredWrench, rightFootTask);
        leftFootTask->setZeroTargetWrench();
        break;
    }

    //updateCoMOpenLoop();
    //updateCoMDistribForce();
    //updateCoMComplianceControl();
    //updateCoMForceTracking();
    //updateCoMZMPCC();
    updateCoMAccelZMPCC();

    updateFootForceDifferenceControl();

    auto endTime = high_resolution_clock::now();
    runTime_ = 1000. * duration_cast<duration<double>>(endTime - startTime).count();
    runTimes_.add(runTime_);
  }

  void Stabilizer::updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd, const sva::ForceVecd & wrench, double leftFootRatio)
  {
    leftFootRatio_ = leftFootRatio;
    measuredCoM_ = com;
    measuredCoMd_ = comd;
    measuredWrench_ = wrench;
  }

  sva::ForceVecd Stabilizer::computeDesiredWrench()
  {
    double omega = pendulum_.omega();
    Eigen::Vector3d comError = pendulum_.com() - measuredCoM_;
    Eigen::Vector3d comdError = pendulum_.comd() - measuredCoMd_;
    dcmError_ = comdError + omega * comError;
    zmpError_ = pendulum_.zmp() - measuredZMP_; // XXX: both in same plane?
    dcmError_.z() = 0.;
    zmpError_.z() = 0.;

    if (!inTheAir_) // don't accumulate error if robot is in the air
    {
      dcmIntegrator_.append(dcmError_);
      dcmAverageError_ = dcmIntegrator_.eval();
    }

    // desired wrench is expressed as CoM acceleration rather than DCM
    // velocity, therefore the proportional gain is offset by omega
    double dcmGainAtCoM = dcmGain_ + omega;

    desiredCoMAccel_ = pendulum_.comdd();
    desiredCoMAccel_ += dcmGainAtCoM * dcmError_;
    desiredCoMAccel_ += dcmIntegralGain_ * dcmAverageError_;
    desiredCoMAccel_ -= zmpGain_ * zmpError_;
    auto desiredForce = mass_ * (desiredCoMAccel_ - world::gravity);
    return {pendulum_.com().cross(desiredForce), desiredForce};
  }

  void Stabilizer::distributeWrench(const sva::ForceVecd & desiredWrench)
  {
    // Variables
    // ---------
    // x = [w_l_0 w_r_0] where
    // w_l_0: spatial force vector of left foot contact in inertial frame
    // w_r_0: spatial force vector of right foot contact in inertial frame
    //
    // Objective
    // ---------
    // Weighted minimization of the following tasks:
    // w_l_0 + w_r_0 == desiredWrench  -- realize desired contact wrench
    // w_l_lankle == 0 -- minimize left foot ankle torque (anisotropic weight)
    // w_r_rankle == 0 -- minimize right foot ankle torque (anisotropic weight)
    // (1 - lfr) * w_l_lc.z() == lfr * w_r_rc.z()
    //
    // Constraints
    // -----------
    // CWC X_0_lc* w_l_0 <= 0  -- left foot wrench within contact wrench cone
    // CWC X_0_rc* w_r_0 <= 0  -- right foot wrench within contact wrench cone
    // (X_0_lc* w_l_0).z() > minPressure  -- minimum left foot contact pressure
    // (X_0_rc* w_r_0).z() > minPressure  -- minimum right foot contact pressure

    const sva::PTransformd & X_0_lc = leftFootContact.pose;
    const sva::PTransformd & X_0_rc = rightFootContact.pose;
    sva::PTransformd X_0_lankle = leftFootContact.anklePose();
    sva::PTransformd X_0_rankle = rightFootContact.anklePose();

    constexpr unsigned NB_VAR = 6 + 6;
    constexpr unsigned COST_DIM = 6 + NB_VAR + 1;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    A.setZero(COST_DIM, NB_VAR);
    b.setZero(COST_DIM);

    // |w_l_0 + w_r_0 - desiredWrench|^2
    auto A_net = A.block<6, 12>(0, 0);
    auto b_net = b.segment<6>(0);
    A_net.block<6, 6>(0, 0) = Eigen::Matrix6d::Identity();
    A_net.block<6, 6>(0, 6) = Eigen::Matrix6d::Identity();
    b_net = desiredWrench.vector();

    // |ankle torques|^2
    auto A_lankle = A.block<6, 6>(6, 0);
    auto A_rankle = A.block<6, 6>(12, 6);
    // anisotropic weights:  taux, tauy, tauz,   fx,   fy,   fz;
    A_lankle.diagonal() <<     1.,   1., 1e-4, 1e-3, 1e-3, 1e-4;
    A_rankle.diagonal() <<     1.,   1., 1e-4, 1e-3, 1e-3, 1e-4;
    A_lankle *= X_0_lankle.dualMatrix();
    A_rankle *= X_0_rankle.dualMatrix();

    // |(1 - lfr) * w_l_lc.force().z() - lfr * w_r_rc.force().z()|^2
    double lfr = leftFootRatio_;
    auto A_pressure = A.block<1, 12>(18, 0);
    A_pressure.block<1, 6>(0, 0) = (1 - lfr) * X_0_lc.dualMatrix().bottomRows<1>();
    A_pressure.block<1, 6>(0, 6) = -lfr * X_0_rc.dualMatrix().bottomRows<1>();

    // Apply weights
    A_net *= fdqpWeights_.netWrenchSqrt;
    b_net *= fdqpWeights_.netWrenchSqrt;
    A_lankle *= fdqpWeights_.ankleTorqueSqrt;
    A_rankle *= fdqpWeights_.ankleTorqueSqrt;
    // b_lankle = 0
    // b_rankle = 0
    A_pressure *= fdqpWeights_.pressureSqrt;
    // b_pressure = 0

    constexpr unsigned CONS_DIM = 16 + 16 + 2;
    Eigen::Matrix<double, CONS_DIM , NB_VAR> C;
    Eigen::VectorXd bl, bu;
    C.setZero(CONS_DIM, NB_VAR);
    bl.setConstant(NB_VAR + CONS_DIM, -1e5);
    bu.setConstant(NB_VAR + CONS_DIM, +1e5);
    auto blCons = bl.tail<CONS_DIM>();
    auto buCons = bu.tail<CONS_DIM>();
    // CWC * w_l_lc <= 0
    C.block<16, 6>(0, 0) = wrenchFaceMatrix_ * X_0_lc.dualMatrix();
    buCons.segment<16>(0).setZero();
    // CWC * w_r_rc <= 0
    C.block<16, 6>(16, 6) = wrenchFaceMatrix_ * X_0_rc.dualMatrix();
    buCons.segment<16>(16).setZero();
    // w_l_lc.force().z() >= MIN_DS_PRESSURE
    // w_r_rc.force().z() >= MIN_DS_PRESSURE
    C.block<1, 6>(32, 0) = X_0_lc.dualMatrix().bottomRows<1>();
    C.block<1, 6>(33, 6) = X_0_rc.dualMatrix().bottomRows<1>();
    blCons.segment<2>(32).setConstant(MIN_DS_PRESSURE);
    buCons.segment<2>(32).setConstant(+1e5);

    //Eigen::MatrixXd A0 = A; // A is modified by solve()
    //Eigen::VectorXd b0 = b; // b is modified by solve()
    bool solverSuccess = wrenchSolver_.solve(A, b, C, bl, bu);
    Eigen::VectorXd x = wrenchSolver_.result();
    if (!solverSuccess)
    {
      LOG_ERROR("DS force distribution QP failed to run");
      wrenchSolver_.print_inform();
      return;
    }

    sva::ForceVecd w_l_0(x.segment<3>(0), x.segment<3>(3));
    sva::ForceVecd w_r_0(x.segment<3>(6), x.segment<3>(9));
    distribWrench_ = w_l_0 + w_r_0;

    sva::ForceVecd w_l_lc = X_0_lc.dualMul(w_l_0);
    sva::ForceVecd w_r_rc = X_0_rc.dualMul(w_r_0);
    Eigen::Vector2d leftCoP = (e_z.cross(w_l_lc.couple()) / w_l_lc.force()(2)).head<2>();
    Eigen::Vector2d rightCoP = (e_z.cross(w_r_rc.couple()) / w_r_rc.force()(2)).head<2>();
    leftFootTask->targetCoP(leftCoP);
    leftFootTask->targetForce(w_l_lc.force());
    rightFootTask->targetCoP(rightCoP);
    rightFootTask->targetForce(w_r_rc.force());
  }

  void Stabilizer::saturateWrench(const sva::ForceVecd & desiredWrench, std::shared_ptr<mc_tasks::CoPTask> & footTask)
  {
    constexpr unsigned NB_CONS = 16;
    constexpr unsigned NB_VAR = 6;

    // Variables
    // ---------
    // x = [w_0] where
    // w_0: spatial force vector of foot contact in inertial frame
    //
    // Objective
    // ---------
    // weighted minimization of |w_c - X_0_c* desiredWrench|^2
    //
    // Constraints
    // -----------
    // F X_0_c* w_0 <= 0    -- contact stability

    const sva::PTransformd & X_0_c = footTask->targetPose();

    Eigen::Matrix6d A = Eigen::Matrix6d::Identity();
    Eigen::Vector6d b = desiredWrench.vector();

    Eigen::MatrixXd C = wrenchFaceMatrix_ * X_0_c.dualMatrix();
    Eigen::VectorXd bl, bu;
    bl.setConstant(NB_VAR + NB_CONS, -1e5);
    bu.setConstant(NB_VAR + NB_CONS, +1e5);
    bu.tail<NB_CONS>().setZero();

    //Eigen::MatrixXd A0 = A; // A is modified by solve()
    //Eigen::VectorXd b0 = b; // b is modified by solve()
    wrenchSolver_.solve(A, b, C, bl, bu);
    Eigen::VectorXd x = wrenchSolver_.result();
    if (wrenchSolver_.inform() != Eigen::lssol::eStatus::STRONG_MINIMUM)
    {
      LOG_ERROR("SS force distribution QP failed to run");
      wrenchSolver_.print_inform();
      return;
    }

    sva::ForceVecd w_0(x.head<3>(), x.tail<3>());
    sva::ForceVecd w_c = X_0_c.dualMul(w_0);
    Eigen::Vector2d cop = (e_z.cross(w_c.couple()) / w_c.force()(2)).head<2>();
    footTask->targetCoP(cop);
    footTask->targetForce(w_c.force());
    distribWrench_ = w_0;
  }

  // void Stabilizer::updateCoMOpenLoop()
  // {
  //   comTask->com(pendulum_.com());
  //   comTask->refVel(pendulum_.comd());
  //   comTask->refAccel(pendulum_.comdd());
  // }

  // void Stabilizer::updateCoMDistribForce()
  // {
  //   comTask->setGains(0., 0.);
  //   comTask->refAccel(distribWrench_.force() / mass_ + world::gravity);
  // }

  // void Stabilizer::updateCoMComplianceControl()
  // {
  //   auto F_error = (distribWrench_ - measuredWrench_).force();
  //   Eigen::Vector3d comAdmittance = {comAdmittance_.x(), comAdmittance_.y(), 0.};
  //   Eigen::Vector3d Delta_com = comAdmittance.cwiseProduct(F_error);
  //   comTask->com(pendulum_.com() + Delta_com);
  //   comTask->refVel(pendulum_.comd());
  //   comTask->refAccel(pendulum_.comdd());
  // }

  // void Stabilizer::updateCoMForceTracking()
  // {
  //   auto forceError = (distribWrench_ - measuredWrench_).force();
  //   Eigen::Vector3d comAdmittance = {comAdmittance_.x(), comAdmittance_.y(), 0.};
  //   Eigen::Vector3d comddFT = comAdmittance.cwiseProduct(forceError);
  //   comTask->com(pendulum_.com());
  //   comTask->refVel(pendulum_.comd());
  //   comTask->refAccel(pendulum_.comdd() + comddFT);
  // }

  // void Stabilizer::updateCoMPosZMPCC()
  // {
  //   auto refForce = mass_ * (pendulum_.comdd() - world::gravity);
  //   sva::ForceVecd refWrench = {pendulum_.com().cross(refForce), refForce};
  //   auto refZMP = computeZMP(refWrench);
  //   auto zmpccError = refZMP - measuredZMP_;
  //   constexpr double T = 0.1;
  //   Eigen::Vector3d comAdmittance = {comAdmittance_.x(), comAdmittance_.y(), 0.};
  //   auto offsetVel = -comAdmittance.cwiseProduct(zmpccError) - zmpccPosOffset / T;
  //   zmpccPosOffset += offsetVel * dt_;
  //   comTask->com(pendulum_.com() + zmpccPosOffset);
  //   comTask->refVel(pendulum_.comd());
  //   comTask->refAccel(pendulum_.comdd());
  // }

  void Stabilizer::updateCoMAccelZMPCC()
  {
    //auto zmpccError = pendulum_.zmp() - measuredZMP_; // nope
    auto distribZMP = computeZMP(distribWrench_);
    auto zmpccError = distribZMP - measuredZMP_; // yes!
    Eigen::Vector3d comAdmittance = {comAdmittance_.x(), comAdmittance_.y(), 0.};
    zmpccAccelOffset_ = -comAdmittance.cwiseProduct(zmpccError);
    comTask->com(pendulum_.com());
    comTask->refVel(pendulum_.comd());
    comTask->refAccel(pendulum_.comdd() + zmpccAccelOffset_);
  }

  void Stabilizer::updateFootForceDifferenceControl()
  {
    double LFz = leftFootTask->measuredWrench().force().z();
    double RFz = rightFootTask->measuredWrench().force().z();
    if (contactState_ == ContactState::DoubleSupport && !inTheAir_)
    {
      double LFz_d = leftFootTask->targetWrench().force().z();
      double RFz_d = rightFootTask->targetWrench().force().z();
      double dz_ctrl = dfzAdmittance_ * ((LFz_d - RFz_d) - (LFz - RFz));

      double LTz = leftFootTask->surfacePose().translation().z();
      double RTz = rightFootTask->surfacePose().translation().z();
      vfcZCtrl_ = RTz - LTz;
      dz_ctrl -= vdcDamping_ * vfcZCtrl_;

      double LTz_d = leftFootTask->targetPose().translation().z();
      double RTz_d = rightFootTask->targetPose().translation().z();
      double dz_pos = vdcFrequency_ * ((LTz_d + RTz_d) - (LTz + RTz));
      vdcZPos_ = RTz + LTz;

      sva::MotionVecd velF = {{0., 0., 0.}, {0., 0., dz_ctrl}};
      sva::MotionVecd velT = {{0., 0., 0.}, {0., 0., dz_pos}};
      leftFootTask->refVelB(0.5 * (velT - velF));
      rightFootTask->refVelB(0.5 * (velT + velF));

      logMeasuredDFz_ = LFz - RFz;
      logMeasuredSTz_ = LTz + RTz;
      logTargetDFz_ = LFz_d - RFz_d;
      logTargetSTz_ = LTz_d + RTz_d;
    }
    else
    {
      leftFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
      rightFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});

      logMeasuredDFz_ = 0.;
      logMeasuredSTz_ = 0.;
      logTargetDFz_ = 0.;
      logTargetSTz_ = 0.;
      vdcZPos_ = 0.;
      vfcZCtrl_ = 0.;
    }
  }
}
