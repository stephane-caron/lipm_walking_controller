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

#include <chrono>
#include <lipm_walking/Stabilizer.h>
#include <lipm_walking/utils/clamp.h>
#include <lipm_walking/utils/world.h>

namespace
{
const Eigen::Vector3d e_z = {0., 0., 1.};
}

namespace lipm_walking
{

// Repeat static constexpr declarations
// Fixes https://github.com/stephane-caron/lipm_walking_controller/issues/21
// See also https://stackoverflow.com/q/8016780
constexpr double Stabilizer::MAX_AVERAGE_DCM_ERROR;
constexpr double Stabilizer::MAX_COM_ADMITTANCE;
constexpr double Stabilizer::MAX_COP_ADMITTANCE;
constexpr double Stabilizer::MAX_DCM_D_GAIN;
constexpr double Stabilizer::MAX_DCM_I_GAIN;
constexpr double Stabilizer::MAX_DCM_P_GAIN;
constexpr double Stabilizer::MAX_DFZ_ADMITTANCE;
constexpr double Stabilizer::MAX_DFZ_DAMPING;
constexpr double Stabilizer::MAX_FDC_RX_VEL;
constexpr double Stabilizer::MAX_FDC_RY_VEL;
constexpr double Stabilizer::MAX_FDC_RZ_VEL;
constexpr double Stabilizer::MAX_ZMPCC_COM_OFFSET;
constexpr double Stabilizer::MIN_DSP_FZ;

namespace
{

inline Eigen::Vector2d vecFromError(const Eigen::Vector3d & error)
{
  double x = -std::round(error.x() * 1000.);
  double y = -std::round(error.y() * 1000.);
  return Eigen::Vector2d{x, y};
}

} // namespace

Stabilizer::Stabilizer(const mc_rbdyn::Robot & controlRobot, const Pendulum & pendulum, double dt)
: dcmIntegrator_(dt, /* timeConstant = */ 5.), dcmDerivator_(dt, /* timeConstant = */ 1.), pendulum_(pendulum),
  controlRobot_(controlRobot), dt_(dt), mass_(controlRobot.mass())
{
}

void Stabilizer::addLogEntries(mc_rtc::Logger & logger)
{
  logger.addLogEntry("stabilizer_contactState", [this]() -> double {
    switch(contactState_)
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
  logger.addLogEntry("error_dcm_average", [this]() { return dcmAverageError_; });
  logger.addLogEntry("error_dcm_pos", [this]() { return dcmError_; });
  logger.addLogEntry("error_dcm_vel", [this]() { return dcmVelError_; });
  logger.addLogEntry("error_dfz_force", [this]() { return dfzForceError_; });
  logger.addLogEntry("error_dfz_height", [this]() { return dfzHeightError_; });
  logger.addLogEntry("error_vdc", [this]() { return vdcHeightError_; });
  logger.addLogEntry("error_zmp", [this]() { return zmpError_; });
  logger.addLogEntry("perf_Stabilizer", [this]() { return runTime_; });
  logger.addLogEntry("stabilizer_admittance_com", [this]() { return comAdmittance_; });
  logger.addLogEntry("stabilizer_admittance_cop", [this]() { return copAdmittance_; });
  logger.addLogEntry("stabilizer_admittance_dfz", [this]() { return dfzAdmittance_; });
  logger.addLogEntry("stabilizer_dcmDerivator", [this]() { return dcmDerivator_.eval(); });
  logger.addLogEntry("stabilizer_dcmDerivator_raw", [this]() { return dcmDerivator_.raw(); });
  logger.addLogEntry("stabilizer_dcmDerivator_timeConstant", [this]() { return dcmDerivator_.timeConstant(); });
  logger.addLogEntry("stabilizer_dcmGains_derivative", [this]() { return dcmDerivGain_; });
  logger.addLogEntry("stabilizer_dcmGains_integral", [this]() { return dcmIntegralGain_; });
  logger.addLogEntry("stabilizer_dcmGains_proportional", [this]() { return dcmPropGain_; });
  logger.addLogEntry("stabilizer_dcmIntegrator", [this]() { return dcmIntegrator_.eval(); });
  logger.addLogEntry("stabilizer_dcmIntegrator_timeConstant", [this]() { return dcmIntegrator_.timeConstant(); });
  logger.addLogEntry("stabilizer_dfz_damping", [this]() { return dfzDamping_; });
  logger.addLogEntry("stabilizer_fdqp_weights_ankleTorque",
                     [this]() { return std::pow(fdqpWeights_.ankleTorqueSqrt, 2); });
  logger.addLogEntry("stabilizer_fdqp_weights_forceRatio",
                     [this]() { return std::pow(fdqpWeights_.forceRatioSqrt, 2); });
  logger.addLogEntry("stabilizer_fdqp_weights_netWrench", [this]() { return std::pow(fdqpWeights_.netWrenchSqrt, 2); });
  logger.addLogEntry("stabilizer_vdc_frequency", [this]() { return vdcFrequency_; });
  logger.addLogEntry("stabilizer_vdc_stiffness", [this]() { return vdcStiffness_; });
  logger.addLogEntry("stabilizer_wrench", [this]() { return distribWrench_; });
  logger.addLogEntry("stabilizer_zmp", [this]() { return zmp(); });
  logger.addLogEntry("stabilizer_zmpcc_comAccel", [this]() { return zmpccCoMAccel_; });
  logger.addLogEntry("stabilizer_zmpcc_comOffset", [this]() { return zmpccCoMOffset_; });
  logger.addLogEntry("stabilizer_zmpcc_comVel", [this]() { return zmpccCoMVel_; });
  logger.addLogEntry("stabilizer_zmpcc_error", [this]() { return zmpccError_; });
  logger.addLogEntry("stabilizer_zmpcc_leakRate", [this]() { return zmpccIntegrator_.rate(); });
}

void Stabilizer::addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
{
  using namespace mc_rtc::gui;
  gui->addElement({"Stabilizer", "Main"}, Button("Disable stabilizer", [this]() { disable(); }),
                  Button("Reconfigure", [this]() { reconfigure(); }),
                  Button("Reset DCM integrator", [this]() { dcmIntegrator_.setZero(); }),
                  ArrayInput("Foot admittance", {"CoPx", "CoPy"},
                             [this]() -> Eigen::Vector2d {
                               return {copAdmittance_.x(), copAdmittance_.y()};
                             },
                             [this](const Eigen::Vector2d & a) {
                               copAdmittance_.x() = clamp(a(0), 0., MAX_COP_ADMITTANCE);
                               copAdmittance_.y() = clamp(a(1), 0., MAX_COP_ADMITTANCE);
                             }),
                  ArrayInput("Foot force difference", {"Admittance", "Damping"},
                             [this]() -> Eigen::Vector2d {
                               return {dfzAdmittance_, dfzDamping_};
                             },
                             [this](const Eigen::Vector2d & a) {
                               dfzAdmittance_ = clamp(a(0), 0., MAX_DFZ_ADMITTANCE);
                               dfzDamping_ = clamp(a(1), 0., MAX_DFZ_DAMPING);
                             }),
                  ArrayInput("DCM gains", {"Prop.", "Integral", "Deriv."},
                             [this]() -> Eigen::Vector3d {
                               return {dcmPropGain_, dcmIntegralGain_, dcmDerivGain_};
                             },
                             [this](const Eigen::Vector3d & gains) {
                               dcmPropGain_ = clamp(gains(0), 0., MAX_DCM_P_GAIN);
                               dcmIntegralGain_ = clamp(gains(1), 0., MAX_DCM_I_GAIN);
                               dcmDerivGain_ = clamp(gains(2), 0., MAX_DCM_D_GAIN);
                             }),
                  ArrayInput("DCM filters", {"Integrator T [s]", "Derivator T [s]"},
                             [this]() -> Eigen::Vector2d {
                               return {dcmIntegrator_.timeConstant(), dcmDerivator_.timeConstant()};
                             },
                             [this](const Eigen::Vector2d & T) {
                               dcmIntegrator_.timeConstant(T(0));
                               dcmDerivator_.timeConstant(T(1));
                             }));
  gui->addElement({"Stabilizer", "Advanced"}, Button("Disable stabilizer", [this]() { disable(); }),
                  Button("Reconfigure", [this]() { reconfigure(); }),
                  Button("Reset CoM integrator", [this]() { zmpccIntegrator_.setZero(); }),
                  Checkbox("Apply CoM admittance only in double support?", [this]() { return zmpccOnlyDS_; },
                           [this]() { zmpccOnlyDS_ = !zmpccOnlyDS_; }),
                  ArrayInput("CoM admittance", {"Ax", "Ay"}, [this]() { return comAdmittance_; },
                             [this](const Eigen::Vector2d & a) {
                               comAdmittance_.x() = clamp(a.x(), 0., MAX_COM_ADMITTANCE);
                               comAdmittance_.y() = clamp(a.y(), 0., MAX_COM_ADMITTANCE);
                             }),
                  NumberInput("CoM integrator leak rate [Hz]", [this]() { return zmpccIntegrator_.rate(); },
                              [this](double T) { zmpccIntegrator_.rate(T); }),
                  ArrayInput("DCM pole placement", {"Pole1", "Pole2", "Pole3", "Lag [Hz]"},
                             [this]() -> Eigen::VectorXd { return polePlacement_; },
                             [this](const Eigen::VectorXd & polePlacement) {
                               double alpha = clamp(polePlacement(0), -20., -0.1);
                               double beta = clamp(polePlacement(1), -20., -0.1);
                               double gamma = clamp(polePlacement(2), -20., -0.1);
                               double lagFreq = clamp(polePlacement(3), 1., 200.);
                               polePlacement_ = {alpha, beta, gamma, lagFreq};

                               double omega = pendulum_.omega();
                               double denom = pendulum_.omega() * lagFreq;
                               double T_integ = dcmIntegrator_.timeConstant();

                               // Gains K_z for the ZMP feedback (Delta ZMP = K_z * Delta DCM)
                               double zmpPropGain =
                                   (alpha * beta + beta * gamma + gamma * alpha + omega * lagFreq) / denom;
                               double zmpIntegralGain = -(alpha * beta * gamma) / denom;
                               double zmpDerivGain = -(alpha + beta + gamma + lagFreq - omega) / denom;

                               // Our gains K are for closed-loop DCM (Delta dot(DCM) = -K * Delta DCM)
                               dcmPropGain_ = omega * (zmpPropGain - 1.);
                               dcmIntegralGain_ = omega * T_integ * zmpIntegralGain; // our integrator is an EMA
                               dcmDerivGain_ = omega * zmpDerivGain;
                             }),
                  ArrayInput("Vertical drift compensation", {"frequency", "stiffness"},
                             [this]() -> Eigen::Vector2d {
                               return {vdcFrequency_, vdcStiffness_};
                             },
                             [this](const Eigen::Vector2d & v) {
                               vdcFrequency_ = clamp(v(0), 0., 10.);
                               vdcStiffness_ = clamp(v(1), 0., 1e4);
                             }),
                  ArrayInput("Wrench distribution weights", {"Net wrench", "Ankle torques", "Force ratio"},
                             [this]() -> Eigen::Vector3d {
                               double netWrench = std::pow(fdqpWeights_.netWrenchSqrt, 2);
                               double ankleTorque = std::pow(fdqpWeights_.ankleTorqueSqrt, 2);
                               double forceRatio = std::pow(fdqpWeights_.forceRatioSqrt, 2);
                               return {netWrench, ankleTorque, forceRatio};
                             },
                             [this](const Eigen::Vector3d & weights) {
                               fdqpWeights_.netWrenchSqrt = std::sqrt(weights(0));
                               fdqpWeights_.ankleTorqueSqrt = std::sqrt(weights(1));
                               fdqpWeights_.forceRatioSqrt = std::sqrt(weights(2));
                             }));
  gui->addElement({"Stabilizer", "Errors"}, Button("Disable stabilizer", [this]() { disable(); }),
                  Button("Reconfigure", [this]() { reconfigure(); }),
                  ArrayLabel("CoM offset [mm]", {"x", "y"}, [this]() { return vecFromError(zmpccCoMOffset_); }),
                  ArrayLabel("DCM average error [mm]", {"x", "y"}, [this]() { return vecFromError(dcmAverageError_); }),
                  ArrayLabel("DCM error [mm]", {"x", "y"}, [this]() { return vecFromError(dcmError_); }),
                  ArrayLabel("Foot force difference error", {"force [N]", "height [mm]"}, [this]() {
                    Eigen::Vector3d dfzError = {dfzForceError_ / 1000., dfzHeightError_, 0.};
                    return vecFromError(dfzError);
                  }));
}

void Stabilizer::disable()
{
  comAdmittance_.setZero();
  copAdmittance_.setZero();
  dcmDerivGain_ = 0.;
  dcmIntegralGain_ = 0.;
  dcmPropGain_ = 0.;
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
  fdqpWeights_.configure(config_("fdqp_weights"));
  if(config_.has("admittance"))
  {
    auto admittance = config_("admittance");
    comAdmittance_ = admittance("com");
    copAdmittance_ = admittance("cop");
    dfzAdmittance_ = admittance("dfz");
    dfzDamping_ = admittance("dfz_damping");
  }
  if(config_.has("dcm_tracking"))
  {
    auto dcmTracking = config_("dcm_tracking");
    dcmPropGain_ = dcmTracking("gains")("prop");
    dcmIntegralGain_ = dcmTracking("gains")("integral");
    dcmDerivGain_ = dcmTracking("gains")("deriv");
    dcmDerivator_.timeConstant(dcmTracking("derivator_time_constant"));
    dcmIntegrator_.timeConstant(dcmTracking("integrator_time_constant"));
  }
  if(config_.has("tasks"))
  {
    auto tasks = config_("tasks");
    if(tasks.has("com"))
    {
      tasks("com")("active_joints", comActiveJoints_);
      tasks("com")("stiffness", comStiffness_);
      tasks("com")("weight", comWeight_);
    }
    if(tasks.has("contact"))
    {
      double d = tasks("contact")("damping");
      double k = tasks("contact")("stiffness");
      contactDamping_ = sva::MotionVecd({d, d, d}, {d, d, d});
      contactStiffness_ = sva::MotionVecd({k, k, k}, {k, k, k});
      tasks("contact")("stiffness", contactStiffness_);
      tasks("contact")("weight", contactWeight_);
    }
    if(tasks.has("swing_foot"))
    {
      tasks("swing_foot")("stiffness", swingFootStiffness_);
      tasks("swing_foot")("weight", swingFootWeight_);
    }
  }
  if(config_.has("vdc"))
  {
    auto vdc = config_("vdc");
    vdcFrequency_ = vdc("frequency");
    vdcStiffness_ = vdc("stiffness");
  }
  if(config_.has("zmpcc"))
  {
    auto zmpcc = config_("zmpcc");
    zmpccIntegrator_.rate(zmpcc("integrator_leak_rate"));
  }
}

void Stabilizer::reset(const mc_rbdyn::Robots & robots)
{
  unsigned robotIndex = robots.robotIndex();

  comTask.reset(new mc_tasks::CoMTask(robots, robotIndex));
  comTask->selectActiveJoints(comActiveJoints_);
  comTask->setGains(comStiffness_, 2 * comStiffness_.cwiseSqrt());
  comTask->weight(comWeight_);

  leftFootTask.reset(new mc_tasks::force::CoPTask("LeftFootCenter", robots, robotIndex));
  rightFootTask.reset(new mc_tasks::force::CoPTask("RightFootCenter", robots, robotIndex));
  leftFootTask->maxAngularVel({MAX_FDC_RX_VEL, MAX_FDC_RY_VEL, MAX_FDC_RZ_VEL});
  rightFootTask->maxAngularVel({MAX_FDC_RX_VEL, MAX_FDC_RY_VEL, MAX_FDC_RZ_VEL});
  setContact(leftFootTask, leftFootTask->surfacePose());
  setContact(rightFootTask, rightFootTask->surfacePose());

  dcmDerivator_.setZero();
  dcmIntegrator_.saturation(MAX_AVERAGE_DCM_ERROR);
  dcmIntegrator_.setZero();
  zmpccIntegrator_.saturation(MAX_ZMPCC_COM_OFFSET);
  zmpccIntegrator_.setZero();

  Eigen::Vector3d staticForce = -mass_ * world::gravity;

  dcmAverageError_ = Eigen::Vector3d::Zero();
  dcmError_ = Eigen::Vector3d::Zero();
  dcmVelError_ = Eigen::Vector3d::Zero();
  dfzForceError_ = 0.;
  dfzHeightError_ = 0.;
  distribWrench_ = {pendulum_.com().cross(staticForce), staticForce};
  vdcHeightError_ = 0.;
  zmpError_ = Eigen::Vector3d::Zero();
  zmpccCoMAccel_ = Eigen::Vector3d::Zero();
  zmpccCoMOffset_ = Eigen::Vector3d::Zero();
  zmpccCoMVel_ = Eigen::Vector3d::Zero();
  zmpccError_ = Eigen::Vector3d::Zero();
}

void Stabilizer::checkGains()
{
  clampInPlace(comAdmittance_.x(), 0., MAX_COM_ADMITTANCE, "CoM x-admittance");
  clampInPlace(comAdmittance_.y(), 0., MAX_COM_ADMITTANCE, "CoM y-admittance");
  clampInPlace(copAdmittance_.x(), 0., MAX_COP_ADMITTANCE, "CoP x-admittance");
  clampInPlace(copAdmittance_.y(), 0., MAX_COP_ADMITTANCE, "CoP y-admittance");
  clampInPlace(dcmDerivGain_, 0., MAX_DCM_D_GAIN, "DCM deriv x-gain");
  clampInPlace(dcmIntegralGain_, 0., MAX_DCM_I_GAIN, "DCM integral x-gain");
  clampInPlace(dcmPropGain_, 0., MAX_DCM_P_GAIN, "DCM prop x-gain");
  clampInPlace(dfzAdmittance_, 0., MAX_DFZ_ADMITTANCE, "DFz admittance");
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

void Stabilizer::setContact(std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact)
{
  footTask->reset();
  footTask->admittance(contactAdmittance());
  footTask->setGains(contactStiffness_, contactDamping_);
  footTask->targetPose(contact.pose);
  footTask->weight(contactWeight_);
  if(footTask->surface() == "LeftFootCenter")
  {
    leftFootContact = contact;
  }
  else if(footTask->surface() == "RightFootCenter")
  {
    rightFootContact = contact;
  }
  else
  {
    LOG_ERROR("Unknown foot surface: " << footTask->surface());
  }
}

void Stabilizer::setSwingFoot(std::shared_ptr<mc_tasks::force::CoPTask> footTask)
{
  footTask->reset();
  footTask->stiffness(swingFootStiffness_); // sets damping as well
  footTask->weight(swingFootWeight_);
}

bool Stabilizer::detectTouchdown(const std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact)
{
  const sva::PTransformd X_0_s = footTask->surfacePose();
  const sva::PTransformd & X_0_c = contact.pose;
  sva::PTransformd X_c_s = X_0_s * X_0_c.inv();
  double xDist = std::abs(X_c_s.translation().x());
  double yDist = std::abs(X_c_s.translation().y());
  double zDist = std::abs(X_c_s.translation().z());
  double Fz = footTask->measuredWrench().force().z();
  return (xDist < 0.03 && yDist < 0.03 && zDist < 0.03 && Fz > 50.);
}

void Stabilizer::seekTouchdown(std::shared_ptr<mc_tasks::force::CoPTask> footTask)
{
  constexpr double MAX_VEL = 0.01; // [m] / [s]
  constexpr double TOUCHDOWN_FORCE = 50.; // [N]
  constexpr double DESIRED_AFZ = MAX_VEL / TOUCHDOWN_FORCE;
  if(footTask->measuredWrench().force().z() < TOUCHDOWN_FORCE)
  {
    auto a = footTask->admittance();
    double AFz = clamp(DESIRED_AFZ, 0., 1e-2, "Contact seeking admittance");
    footTask->admittance({a.couple(), {a.force().x(), a.force().y(), AFz}});
    footTask->targetForce({0., 0., TOUCHDOWN_FORCE});
  }
}

void Stabilizer::setSupportFootGains()
{
  sva::MotionVecd vdcContactStiffness = {contactStiffness_.angular(), {vdcStiffness_, vdcStiffness_, vdcStiffness_}};
  switch(contactState_)
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
  inTheAir_ = (LFz < MIN_DSP_FZ && RFz < MIN_DSP_FZ);
}

void Stabilizer::updateZMPFrame()
{
  const sva::PTransformd & X_0_lc = leftFootContact.pose;
  const sva::PTransformd & X_0_rc = rightFootContact.pose;
  switch(contactState_)
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
  double normalForce = n.dot(force);
  if(normalForce < 1.)
  {
    double lambda = std::pow(pendulum_.omega(), 2);
    return measuredCoM_ + world::gravity / lambda; // default for logging
  }
  const Eigen::Vector3d & moment_0 = wrench.couple();
  Eigen::Vector3d moment_p = moment_0 - p.cross(force);
  return p + n.cross(moment_p) / normalForce;
}

void Stabilizer::run()
{
  auto startTime = std::chrono::high_resolution_clock::now();
  checkGains();
  checkInTheAir();
  setSupportFootGains();
  updateZMPFrame();
  auto desiredWrench = computeDesiredWrench();
  switch(contactState_)
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
  updateCoMTaskZMPCC();
  updateFootForceDifferenceControl();
  auto endTime = std::chrono::high_resolution_clock::now();
  runTime_ = std::chrono::duration<double, std::milli>(endTime - startTime).count();
}

void Stabilizer::updateState(const Eigen::Vector3d & com,
                             const Eigen::Vector3d & comd,
                             const sva::ForceVecd & wrench,
                             double leftFootRatio)
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
  dcmError_ = comError + comdError / omega;
  dcmError_.z() = 0.;

  if(inTheAir_)
  {
    dcmDerivator_.setZero();
    dcmIntegrator_.append(Eigen::Vector3d::Zero());
  }
  else
  {
    zmpError_ = pendulum_.zmp() - measuredZMP_; // XXX: both in same plane?
    zmpError_.z() = 0.;
    dcmDerivator_.update(omega * (dcmError_ - zmpError_));
    dcmIntegrator_.append(dcmError_);
  }
  dcmAverageError_ = dcmIntegrator_.eval();
  dcmVelError_ = dcmDerivator_.eval();

  Eigen::Vector3d desiredCoMAccel = pendulum_.comdd();
  desiredCoMAccel += omega * (dcmPropGain_ * dcmError_ + comdError);
  desiredCoMAccel += omega * dcmIntegralGain_ * dcmAverageError_;
  desiredCoMAccel += omega * dcmDerivGain_ * dcmVelError_;
  auto desiredForce = mass_ * (desiredCoMAccel - world::gravity);

  // Previous implementation (up to v1.3):
  // return {pendulum_.com().cross(desiredForce), desiredForce};
  // See https://github.com/stephane-caron/lipm_walking_controller/issues/28

  return {measuredCoM_.cross(desiredForce), desiredForce};
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
  // (X_0_lc* w_l_0).z() > MIN_DSP_FZ  -- minimum left foot contact force
  // (X_0_rc* w_r_0).z() > MIN_DSP_FZ  -- minimum right foot contact force

  const sva::PTransformd & X_0_lc = leftFootContact.pose;
  const sva::PTransformd & X_0_rc = rightFootContact.pose;
  sva::PTransformd X_0_lankle = leftFootContact.anklePose(sole_);
  sva::PTransformd X_0_rankle = rightFootContact.anklePose(sole_);

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
  A_lankle.diagonal() << 1., 1., 1e-4, 1e-3, 1e-3, 1e-4;
  A_rankle.diagonal() << 1., 1., 1e-4, 1e-3, 1e-3, 1e-4;
  A_lankle *= X_0_lankle.dualMatrix();
  A_rankle *= X_0_rankle.dualMatrix();

  // |(1 - lfr) * w_l_lc.force().z() - lfr * w_r_rc.force().z()|^2
  double lfr = leftFootRatio_;
  auto A_fratio = A.block<1, 12>(18, 0);
  A_fratio.block<1, 6>(0, 0) = (1 - lfr) * X_0_lc.dualMatrix().bottomRows<1>();
  A_fratio.block<1, 6>(0, 6) = -lfr * X_0_rc.dualMatrix().bottomRows<1>();

  // Apply weights
  A_net *= fdqpWeights_.netWrenchSqrt;
  b_net *= fdqpWeights_.netWrenchSqrt;
  A_lankle *= fdqpWeights_.ankleTorqueSqrt;
  A_rankle *= fdqpWeights_.ankleTorqueSqrt;
  // b_lankle = 0
  // b_rankle = 0
  A_fratio *= fdqpWeights_.forceRatioSqrt;
  // b_fratio = 0

  Eigen::MatrixXd Q = A.transpose() * A;
  Eigen::VectorXd c = -A.transpose() * b;

  constexpr unsigned NB_CONS = 16 + 16 + 2;
  Eigen::Matrix<double, NB_CONS, NB_VAR> A_ineq;
  Eigen::VectorXd b_ineq;
  A_ineq.setZero(NB_CONS, NB_VAR);
  b_ineq.setZero(NB_CONS);
  // CWC * w_l_lc <= 0
  A_ineq.block<16, 6>(0, 0) = wrenchFaceMatrix_ * X_0_lc.dualMatrix();
  // b_ineq.segment<16>(0) is already zero
  // CWC * w_r_rc <= 0
  A_ineq.block<16, 6>(16, 6) = wrenchFaceMatrix_ * X_0_rc.dualMatrix();
  // b_ineq.segment<16>(16) is already zero
  // w_l_lc.force().z() >= MIN_DSP_FZ
  A_ineq.block<1, 6>(32, 0) = -X_0_lc.dualMatrix().bottomRows<1>();
  b_ineq(32) = -MIN_DSP_FZ;
  // w_r_rc.force().z() >= MIN_DSP_FZ
  A_ineq.block<1, 6>(33, 6) = -X_0_rc.dualMatrix().bottomRows<1>();
  b_ineq(33) = -MIN_DSP_FZ;

  qpSolver_.problem(NB_VAR, 0, NB_CONS);
  Eigen::MatrixXd A_eq(0, 0);
  Eigen::VectorXd b_eq;
  b_eq.resize(0);

  bool solutionFound = qpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq, /* isDecomp = */ false);
  if(!solutionFound)
  {
    LOG_ERROR("DS force distribution QP: solver found no solution");
    return;
  }

  Eigen::VectorXd x = qpSolver_.result();
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

void Stabilizer::saturateWrench(const sva::ForceVecd & desiredWrench,
                                std::shared_ptr<mc_tasks::force::CoPTask> & footTask)
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

  Eigen::Matrix6d Q = Eigen::Matrix6d::Identity();
  Eigen::Vector6d c = -desiredWrench.vector();

  Eigen::MatrixXd A_ineq = wrenchFaceMatrix_ * X_0_c.dualMatrix();
  Eigen::VectorXd b_ineq;
  b_ineq.setZero(NB_CONS);

  qpSolver_.problem(NB_VAR, 0, NB_CONS);
  Eigen::MatrixXd A_eq(0, 0);
  Eigen::VectorXd b_eq;
  b_eq.resize(0);

  bool solutionFound = qpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq, /* isDecomp = */ true);
  if(!solutionFound)
  {
    LOG_ERROR("SS force distribution QP: solver found no solution");
    return;
  }

  Eigen::VectorXd x = qpSolver_.result();
  sva::ForceVecd w_0(x.head<3>(), x.tail<3>());
  sva::ForceVecd w_c = X_0_c.dualMul(w_0);
  Eigen::Vector2d cop = (e_z.cross(w_c.couple()) / w_c.force()(2)).head<2>();
  footTask->targetCoP(cop);
  footTask->targetForce(w_c.force());
  distribWrench_ = w_0;
}

void Stabilizer::updateCoMTaskZMPCC()
{
  if(zmpccOnlyDS_ && contactState_ != ContactState::DoubleSupport)
  {
    zmpccCoMAccel_.setZero();
    zmpccCoMVel_.setZero();
    zmpccIntegrator_.add(Eigen::Vector3d::Zero(), dt_); // leak to zero
  }
  else
  {
    auto distribZMP = computeZMP(distribWrench_);
    zmpccError_ = distribZMP - measuredZMP_;
    const Eigen::Matrix3d & R_0_c = zmpFrame_.rotation();
    const Eigen::Transpose<const Eigen::Matrix3d> R_c_0 = R_0_c.transpose();
    Eigen::Vector3d comAdmittance = {comAdmittance_.x(), comAdmittance_.y(), 0.};
    Eigen::Vector3d newVel = -R_c_0 * comAdmittance.cwiseProduct(R_0_c * zmpccError_);
    Eigen::Vector3d newAccel = (newVel - zmpccCoMVel_) / dt_;
    zmpccIntegrator_.add(newVel, dt_);
    zmpccCoMAccel_ = newAccel;
    zmpccCoMVel_ = newVel;
  }
  zmpccCoMOffset_ = zmpccIntegrator_.eval();
  comTask->com(pendulum_.com() + zmpccCoMOffset_);
  comTask->refVel(pendulum_.comd() + zmpccCoMVel_);
  comTask->refAccel(pendulum_.comdd() + zmpccCoMAccel_);
}

void Stabilizer::updateFootForceDifferenceControl()
{
  if(contactState_ != ContactState::DoubleSupport || inTheAir_)
  {
    dfzForceError_ = 0.;
    dfzHeightError_ = 0.;
    vdcHeightError_ = 0.;
    leftFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
    rightFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
    return;
  }

  double LFz_d = leftFootTask->targetWrench().force().z();
  double RFz_d = rightFootTask->targetWrench().force().z();
  double LFz = leftFootTask->measuredWrench().force().z();
  double RFz = rightFootTask->measuredWrench().force().z();
  dfzForceError_ = (LFz_d - RFz_d) - (LFz - RFz);

  double LTz_d = leftFootTask->targetPose().translation().z();
  double RTz_d = rightFootTask->targetPose().translation().z();
  double LTz = leftFootTask->surfacePose().translation().z();
  double RTz = rightFootTask->surfacePose().translation().z();
  dfzHeightError_ = (LTz_d - RTz_d) - (LTz - RTz);
  vdcHeightError_ = (LTz_d + RTz_d) - (LTz + RTz);

  double dz_ctrl = dfzAdmittance_ * dfzForceError_ - dfzDamping_ * dfzHeightError_;
  double dz_vdc = vdcFrequency_ * vdcHeightError_;
  sva::MotionVecd velF = {{0., 0., 0.}, {0., 0., dz_ctrl}};
  sva::MotionVecd velT = {{0., 0., 0.}, {0., 0., dz_vdc}};
  leftFootTask->refVelB(0.5 * (velT - velF));
  rightFootTask->refVelB(0.5 * (velT + velF));
}

} // namespace lipm_walking
