/* Copyright 2018 CNRS-UM LIRMM
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

#include <lipm_walking/Stabilizer.h>
#include <lipm_walking/utils/clamp.h>

namespace lipm_walking
{
  Stabilizer::Stabilizer(const mc_rbdyn::Robot & controlRobot, const Pendulum & pendulum, double dt, std::shared_ptr<mc_tasks::PostureTask> postureTask_)
    : postureTask(postureTask_),
      pendulum_(pendulum),
      controlRobot_(controlRobot),
      dt_(dt),
      mass_(controlRobot.mass()),
      distribWrench_(Eigen::Vector6d::Zero())
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
    logger.addLogEntry("stabilizer_distrib_zmp", [this]() { return distribZMP(); });
    logger.addLogEntry("stabilizer_errors_com", [this]() { return comError_; });
    logger.addLogEntry("stabilizer_errors_comd", [this]() { return comdError_; });
    logger.addLogEntry("stabilizer_errors_dcm", [this]() { return dcmError_; });
    logger.addLogEntry("stabilizer_errors_dcmi", [this]() { return dcmIntegralError_; });
    logger.addLogEntry("stabilizer_gains_com_admittance", [this]() { return comAdmittance_; });
    logger.addLogEntry("stabilizer_gains_com_stiffness", [this]() { return comStiffness_; });
    logger.addLogEntry("stabilizer_gains_contact_admittance", [this]() { return contactAdmittance_; });
    logger.addLogEntry("stabilizer_gains_dcm", [this]() { return dcmGain_; });
    logger.addLogEntry("stabilizer_gains_dcmi", [this]() { return dcmIntegralGain_; });
    logger.addLogEntry("stabilizer_gains_dfz_admittance", [this]() { return dfzAdmittance_; });
    logger.addLogEntry("stabilizer_gains_vdc_frequency", [this]() { return vdcFrequency_; });
    logger.addLogEntry("stabilizer_gains_vdc_stiffness", [this]() { return vdcStiffness_; });
    logger.addLogEntry("stabilizer_integrator_decay", [this]() { return dcmIntegrator.decay(); });
    logger.addLogEntry("stabilizer_qp_costs_left_ankle", [this]() { return qpLeftAnkleCost_; });
    logger.addLogEntry("stabilizer_qp_costs_net_wrench", [this]() { return qpNetWrenchCost_; });
    logger.addLogEntry("stabilizer_qp_costs_pressure_ratio", [this]() { return qpPressureCost_; });
    logger.addLogEntry("stabilizer_qp_costs_right_ankle", [this]() { return qpRightAnkleCost_; });
    logger.addLogEntry("stabilizer_qp_weights_compliance", [this]() { return std::pow(qpWeights_.complianceSqrt, 2); });
    logger.addLogEntry("stabilizer_qp_weights_net_wrench", [this]() { return std::pow(qpWeights_.netWrenchSqrt, 2); });
    logger.addLogEntry("stabilizer_qp_weights_pressure", [this]() { return std::pow(qpWeights_.pressureSqrt, 2); });
    logger.addLogEntry("stabilizer_torso_pitch", [this]() { return torsoPitch_; });
    logger.addLogEntry("stabilizer_vfc_LeftFootVel", [this]() { return logVFCLeftFootVel_; });
    logger.addLogEntry("stabilizer_vfc_RightFootVel", [this]() { return logVFCRightFootVel_; });
    logger.addLogEntry("stabilizer_vfc_dfz_measured", [this]() { return logMeasuredDFz_; });
    logger.addLogEntry("stabilizer_vfc_dfz_target", [this]() { return logTargetDFz_; });
    logger.addLogEntry("stabilizer_vfc_stz_measured", [this]() { return logMeasuredSTz_; });
    logger.addLogEntry("stabilizer_vfc_stz_target", [this]() { return logTargetSTz_; });
    logger.addLogEntry("stabilizer_zmpcc_comdd_offset", [this]() { return zmpccAccelOffset_; });
  }

  void Stabilizer::addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
  {
    using namespace mc_rtc::gui;
    gui->addElement(
      {"Walking", "Stabilizer"},
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
              return "RightFoot";
            case ContactState::Flying:
            default:
              return "Flying";
          }
        }),
      ArrayInput(
        "CoM admittance", {"Cx", "Cy", "Cz"},
        [this]() { return comAdmittance_; },
        [this](const Eigen::Vector3d & c) { comAdmittance_ = c; }),
      ArrayInput(
        "CoM stiffness", {"Kx", "Ky", "Kz"},
        [this]() { return comStiffness_; },
        [this](const Eigen::Vector3d & k) { comStiffness_ = k; }),
      ArrayInput(
        "CoP admittance", {"Ax", "Ay"},
        [this]() -> Eigen::Vector2d
        {
          double a_cop_x = contactAdmittance_.couple().y();
          double a_cop_y = contactAdmittance_.couple().x();
          return {a_cop_x, a_cop_y};
        },
        [this](const Eigen::Vector2d & a)
        {
          double a_cop_x = clamp(a(0), 0., MAX_COP_ADMITTANCE_X);
          double a_cop_y = clamp(a(1), 0., MAX_COP_ADMITTANCE_Y);
          contactAdmittance_ = {{a_cop_y, a_cop_x, 0.}, {0., 0., 0.}};
        }),
      NumberInput(
        "DFz Admittance",
        [this]() { return dfzAdmittance_; },
        [this](double a) { dfzAdmittance_ = clamp(a, 0., MAX_DFZ_ADMITTANCE); }),
      ArrayInput(
        "Vertical Drift Control", {"frequency", "stiffness"},
        [this]() -> Eigen::Vector2d { return {vdcFrequency_, vdcStiffness_}; },
        [this](const Eigen::Vector2d & v)
        {
          vdcFrequency_ = v(0);
          vdcStiffness_ = v(1);
        }),
      NumberInput(
        "DCM Gain",
        [this]() { return dcmGain_; },
        [this](double k_p) { dcmGain_ = clamp(k_p, 0., MAX_DCM_P_GAIN); }),
      NumberInput(
        "DCM Integral Decay",
        [this]() { return dcmIntegrator.decay(); },
        [this](double decay) { return dcmIntegrator.decay(decay); }),
      NumberInput(
        "DCM Integral Gain",
        [this]() { return dcmIntegralGain_; },
        [this](double k_i) { dcmIntegralGain_ = clamp(k_i, 0., MAX_DCM_I_GAIN); }),
      NumberInput(
        "Torso pitch [rad]",
        [this]() { return torsoPitch_; },
        [this](double pitch) { torsoPitch_ = clamp(pitch, -0.5, 0.5); }),
      Button(
        "Reconfigure",
        [this]()
        {
          reconfigure();
          //setSupportFootGains(); // now continuously updated
        }),
      Button(
        "Reset DCM integrator",
        [this]()
        {
          dcmIntegrator.reset();
        }),
      Button(
        "Disable",
        [this]() { disable(); })
    );
  }

  void Stabilizer::disable()
  {
    comAdmittance_ = {0., 0., 0.};
    contactAdmittance_ = {{0., 0., 0.}, {0., 0., 0.}};
    dcmGain_ = 0.;
    dcmIntegralGain_ = 0.;
    dfzAdmittance_ = 0.;
    vdcFrequency_ = 0.;
    vdcStiffness_ = 0.;
  }

  void Stabilizer::configure(const mc_rtc::Configuration & config)
  {
    config_ = config;
    reconfigure();
  }

  void Stabilizer::reconfigure()
  {
    comAdmittance_ = config_("com_admittance");
    dfzAdmittance_ = config_("dfz_admittance");
    torsoPitch_ = config_("torso_pitch");
    vdcFrequency_ = config_("vdc_frequency");
    vdcStiffness_ = config_("vdc_stiffness");
    if (config_.has("dcm_tracking"))
    {
      auto dcmConfig = config_("dcm_tracking");
      dcmConfig("gain", dcmGain_);
      dcmConfig("integral_gain", dcmIntegralGain_);
      if (dcmConfig.has("integrator_decay"))
      {
        dcmIntegrator.decay(dcmConfig("integrator_decay"));
      }
    }
    if (config_.has("tasks"))
    {
      auto tasks = config_("tasks");
      if (tasks.has("com"))
      {
        tasks("com")("stiffness", comStiffness_);
        tasks("com")("weight", comWeight_);
      }
      if (tasks.has("contact"))
      {
        tasks("contact")("admittance", contactAdmittance_);
        tasks("contact")("damping", contactDamping_);
        tasks("contact")("stiffness", contactStiffness_);
        tasks("contact")("weight", contactWeight_);
      }
      if (tasks.has("pelvis"))
      {
        tasks("pelvis")("stiffness", pelvisStiffness_);
        tasks("pelvis")("weight", pelvisWeight_);
      }
      if (tasks.has("posture"))
      {
        tasks("posture")("stiffness", postureStiffness_);
        tasks("posture")("weight", postureWeight_);
      }
      if (tasks.has("swing_foot"))
      {
        tasks("swing_foot")("stiffness", swingFootStiffness_);
        tasks("swing_foot")("weight", swingFootWeight_);
      }
      if (tasks.has("torso"))
      {
        tasks("torso")("stiffness", torsoStiffness_);
        tasks("torso")("weight", torsoWeight_);
      }
    }
  }

  void Stabilizer::reset(const mc_rbdyn::Robots & robots)
  {
    unsigned robotIndex = robots.robotIndex();

    comTask.reset(new mc_tasks::CoMTask(robots, robotIndex));
    comTask->selectActiveJoints({
        "Root",
        "R_HIP_Y", "R_HIP_R", "R_HIP_P", "R_KNEE_P", "R_ANKLE_P", "R_ANKLE_R",
        "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE_P", "L_ANKLE_P", "L_ANKLE_R"
        });
    comTask->setGains(comStiffness_, 2 * comStiffness_.cwiseSqrt());
    comTask->weight(comWeight_);

    leftFootTask.reset(new mc_tasks::CoPTask("LeftFootCenter", robots, robotIndex));
    rightFootTask.reset(new mc_tasks::CoPTask("RightFootCenter", robots, robotIndex));
    leftFootTask->maxAngularVel({MAX_FDC_RX_VEL, MAX_FDC_RY_VEL, MAX_FDC_RZ_VEL});
    rightFootTask->maxAngularVel({MAX_FDC_RX_VEL, MAX_FDC_RY_VEL, MAX_FDC_RZ_VEL});
    setContact(leftFootTask, leftFootTask->surfacePose());
    setContact(rightFootTask, rightFootTask->surfacePose());

    pelvisTask.reset(new mc_tasks::OrientationTask("base_link", robots, robotIndex));
    pelvisTask->setGains(pelvisStiffness_, 2 * std::sqrt(pelvisStiffness_));
    pelvisTask->weight(pelvisWeight_);

    torsoTask.reset(new mc_tasks::OrientationTask("torso", robots, robotIndex));
    torsoTask->setGains(torsoStiffness_, 2 * std::sqrt(torsoStiffness_));
    torsoTask->weight(torsoWeight_);

    postureTask->stiffness(postureStiffness_);
    postureTask->weight(postureWeight_);

    dcmIntegrator.reset();
    dcmIntegrator.saturation(0.5);
  }

  void Stabilizer::checkGains()
  {
    clampInPlace(comAdmittance_.x(), 0., MAX_COM_ADMITTANCE_X, "CoM a_x");
    clampInPlace(comAdmittance_.y(), 0., MAX_COM_ADMITTANCE_Y, "CoM a_y");
    clampInPlace(contactAdmittance_.couple().x(), 0., MAX_COP_ADMITTANCE_Y, "CoP a_y");
    clampInPlace(contactAdmittance_.couple().y(), 0., MAX_COP_ADMITTANCE_X, "CoP a_x");
    clampInPlace(dcmGain_, 0., MAX_DCM_P_GAIN, "DCM k_p");
    clampInPlace(dcmIntegralGain_, 0., MAX_DCM_I_GAIN, "DCM k_i");
    clampInPlace(dfzAdmittance_, 0., MAX_DFZ_ADMITTANCE, "DFz a");
  }

  void Stabilizer::addTasks(mc_solver::QPSolver & solver)
  {
    solver.addTask(comTask);
    solver.addTask(leftFootTask);
    solver.addTask(pelvisTask);
    solver.addTask(rightFootTask);
    solver.addTask(torsoTask);
  }

  void Stabilizer::removeTasks(mc_solver::QPSolver & solver)
  {
    solver.removeTask(comTask);
    solver.removeTask(leftFootTask);
    solver.removeTask(pelvisTask);
    solver.removeTask(rightFootTask);
    solver.removeTask(torsoTask);
  }

  void Stabilizer::setContact(std::shared_ptr<mc_tasks::CoPTask> footTask, const Contact & contact)
  {
    footTask->reset();
    footTask->admittance(contactAdmittance_);
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
      //{contactStiffness_.linear().x(), contactStiffness_.linear().y(), vdcStiffness_}};
      {vdcStiffness_, vdcStiffness_, vdcStiffness_}};
    switch (contactState_)
    {
      case ContactState::DoubleSupport:
        leftFootTask->admittance(contactAdmittance_);
        leftFootTask->setGains(contactStiffness_, contactDamping_);
        rightFootTask->admittance(contactAdmittance_);
        rightFootTask->setGains(contactStiffness_, contactDamping_);
        break;
      case ContactState::LeftFoot:
        leftFootTask->admittance(contactAdmittance_);
        leftFootTask->setGains(vdcContactStiffness, contactDamping_);
        break;
      case ContactState::RightFoot:
        rightFootTask->admittance(contactAdmittance_);
        rightFootTask->setGains(vdcContactStiffness, contactDamping_);
        break;
      case ContactState::Flying:
        break;
    }
  }

  void Stabilizer::run()
  {
    checkGains();
    setSupportFootGains();
    updatePelvis();

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
      case ContactState::Flying:
        break;
    }

    auto comDamping = 2 * comStiffness_.cwiseSqrt();
    comTask->setGains(comStiffness_, comDamping);
    //updateCoMOpenLoop();
    //updateCoMDistribForce();
    //updateCoMComplianceControl();
    //updateCoMForceTracking();
    //updateCoMZMPCC();
    updateCoMAccelZMPCC();

    updateFootForceDifferenceControl();
  }

  void Stabilizer::updatePelvis()
  {
    const sva::PTransformd & leftPose = leftFootContact.pose;
    const sva::PTransformd & rightPose = rightFootContact.pose;
    sva::PTransformd target;
    switch (contactState_)
    {
      case ContactState::DoubleSupport:
        target = sva::interpolate(leftPose, rightPose, 0.5);
        break;
      case ContactState::LeftFoot:
        target = leftPose;
        break;
      case ContactState::RightFoot:
        target = rightPose;
        break;
      default:
        target = sva::PTransformd::Identity();
        break;
    }
    pelvisTask->orientation(target.rotation());

    Eigen::Matrix3d E_pitch = mc_rbdyn::rpyToMat(0., torsoPitch_, 0.);
    torsoTask->orientation(E_pitch * target.rotation());
  }

  sva::ForceVecd Stabilizer::computeDesiredWrench()
  {
    comError_ = pendulum_.com() - measuredCoM_;
    comdError_ = pendulum_.comd() - measuredCoMd_;

    double omega = pendulum_.omega();
    dcmError_ = comdError_ + omega * comError_;

    dcmIntegrator.add(dcmError_, dt_);
    dcmIntegralError_ = dcmIntegrator.eval();

    desiredCoMAccel_ = pendulum_.comdd();
    desiredCoMAccel_ += dcmGain_ * dcmError_;
    desiredCoMAccel_ += dcmIntegralGain_ * dcmIntegralError_;
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
    A_net *= qpWeights_.netWrenchSqrt;
    b_net *= qpWeights_.netWrenchSqrt;
    A_lankle *= qpWeights_.complianceSqrt;
    A_rankle *= qpWeights_.complianceSqrt;
    // b_lankle = 0
    // b_rankle = 0
    A_pressure *= qpWeights_.pressureSqrt;
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

    Eigen::MatrixXd A0 = A; // A is modified by solve()
    Eigen::VectorXd b0 = b; // b is modified by solve()
    wrenchSolver_.solve(A, b, C, bl, bu);
    Eigen::VectorXd x = wrenchSolver_.result();
    if (wrenchSolver_.inform() != Eigen::lssol::eStatus::STRONG_MINIMUM)
    {
      LOG_ERROR("DS force distribution QP failed to run");
      wrenchSolver_.print_inform();
      return;
    }

    auto error = A0 * x - b0;
    qpNetWrenchCost_ = error.segment<6>(0).norm() / qpWeights_.netWrenchSqrt;
    qpLeftAnkleCost_ = error.segment<6>(6).norm() / qpWeights_.complianceSqrt;
    qpRightAnkleCost_ = error.segment<6>(12).norm() / qpWeights_.complianceSqrt;
    qpPressureCost_ = error.segment<1>(13).norm() / qpWeights_.pressureSqrt;

    sva::ForceVecd w_l_0(x.segment<3>(0), x.segment<3>(3));
    sva::ForceVecd w_r_0(x.segment<3>(6), x.segment<3>(9));
    outputFrame_ = sva::interpolate(X_0_lc, X_0_rc, 0.5);
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

    Eigen::MatrixXd A0 = A; // A is modified by solve()
    Eigen::VectorXd b0 = b; // b is modified by solve()
    wrenchSolver_.solve(A, b, C, bl, bu);
    Eigen::VectorXd x = wrenchSolver_.result();
    if (wrenchSolver_.inform() != Eigen::lssol::eStatus::STRONG_MINIMUM)
    {
      LOG_ERROR("SS force distribution QP failed to run");
      wrenchSolver_.print_inform();
      return;
    }

    qpNetWrenchCost_ = (A0 * x - b0).norm();
    qpLeftAnkleCost_ = 0.;
    qpRightAnkleCost_ = 0.;
    qpPressureCost_ = 0.;

    sva::ForceVecd w_0(x.head<3>(), x.tail<3>());
    sva::ForceVecd w_c = X_0_c.dualMul(w_0);
    Eigen::Vector2d cop = (e_z.cross(w_c.couple()) / w_c.force()(2)).head<2>();
    footTask->targetCoP(cop);
    footTask->targetForce(w_c.force());
    outputFrame_ = X_0_c;
    distribWrench_ = w_0;
  }

  Eigen::Vector3d Stabilizer::computeOutputFrameZMP(const sva::ForceVecd & wrench) const
  {
    Eigen::Vector3d n = outputFrame_.rotation().row(2);
    Eigen::Vector3d p = outputFrame_.translation();
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
  //   Eigen::Vector3d Delta_com = comAdmittance_.cwiseProduct(F_error);
  //   comTask->com(pendulum_.com() + Delta_com);
  //   comTask->refVel(pendulum_.comd());
  //   comTask->refAccel(pendulum_.comdd());
  // }

  //void Stabilizer::updateCoMForceTracking()
  //{
  //  auto forceError = (distribWrench_ - measuredWrench_).force();
  //  Eigen::Vector3d comddFT = comAdmittance_.cwiseProduct(forceError);
  //  comTask->com(pendulum_.com());
  //  comTask->refVel(pendulum_.comd());
  //  comTask->refAccel(pendulum_.comdd() + comddFT);
  //}

  //void Stabilizer::updateCoMPosZMPCC()
  //{
  //  auto refForce = mass_ * (pendulum_.comdd() - world::gravity);
  //  sva::ForceVecd refWrench = {pendulum_.com().cross(refForce), refForce};
  //  auto refZMP = computeOutputFrameZMP(refWrench);
  //  auto measuredZMP = computeOutputFrameZMP(measuredWrench_);
  //  auto zmpError = refZMP - measuredZMP;
  //  constexpr double T = 0.1;
  //  auto offsetVel = -comAdmittance_.cwiseProduct(zmpError) - zmpccPosOffset / T;
  //  zmpccPosOffset += offsetVel * dt_;
  //  comTask->com(pendulum_.com() + zmpccPosOffset);
  //  comTask->refVel(pendulum_.comd());
  //  comTask->refAccel(pendulum_.comdd());
  //}

  void Stabilizer::updateCoMAccelZMPCC()
  {
    auto measuredZMP = computeOutputFrameZMP(measuredWrench_);
    //auto zmpError = pendulum_.zmp() - measuredZMP; // nope
    auto distribZMP = computeOutputFrameZMP(distribWrench_);
    auto zmpError = distribZMP - measuredZMP; // yes!
    zmpccAccelOffset_ = -comAdmittance_.cwiseProduct(zmpError);
    comTask->com(pendulum_.com());
    comTask->refVel(pendulum_.comd());
    comTask->refAccel(pendulum_.comdd() + zmpccAccelOffset_);
  }

  void Stabilizer::updateFootForceDifferenceControl()
  {
    double LFz = leftFootTask->measuredWrench().force().z();
    double RFz = rightFootTask->measuredWrench().force().z();
    bool inTheAir = (LFz < MIN_DS_PRESSURE && RFz < MIN_DS_PRESSURE);
    if (contactState_ == ContactState::DoubleSupport && !inTheAir)
    {
      double LFz_d = leftFootTask->targetWrench().force().z();
      double RFz_d = rightFootTask->targetWrench().force().z();
      double dz_ctrl = dfzAdmittance_ * ((LFz_d - RFz_d) - (LFz - RFz));
      sva::MotionVecd velF = {{0., 0., 0.}, {0., 0., dz_ctrl}};

      double LTz = leftFootTask->surfacePose().translation().z();
      double RTz = rightFootTask->surfacePose().translation().z();
      double LTz_d = leftFootTask->targetPose().translation().z();
      double RTz_d = rightFootTask->targetPose().translation().z();
      double dz_pos = vdcFrequency_ * ((LTz_d + RTz_d) - (LTz + RTz));
      sva::MotionVecd velT = {{0., 0., 0.}, {0., 0., dz_pos}};

      leftFootTask->refVelB(0.5 * (velT - velF));
      rightFootTask->refVelB(0.5 * (velT + velF));

      logTargetDFz_ = LFz_d - RFz_d;
      logMeasuredDFz_ = LFz - RFz;
      logTargetSTz_ = LTz_d + RTz_d;
      logMeasuredSTz_ = LTz + RTz;
      logVFCLeftFootVel_ = 0.5 * (dz_pos - dz_ctrl);
      logVFCRightFootVel_ = 0.5 * (dz_pos + dz_ctrl);
    }
    else
    {
      leftFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
      rightFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});

      logTargetDFz_ = 0.;
      logMeasuredDFz_ = 0.;
      logTargetSTz_ = 0.;
      logMeasuredSTz_ = 0.;
      logVFCLeftFootVel_ = 0.;
      logVFCRightFootVel_ = 0.;
    }
  }
}
