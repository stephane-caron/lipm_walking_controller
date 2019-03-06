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

#include <iomanip>

#include <lipm_walking/HorizontalMPCProblem.h>
#include <lipm_walking/utils/clamp.h>

namespace lipm_walking
{
  using namespace HorizontalMPC;

  HorizontalMPCProblem::HorizontalMPCProblem()
  {
    velFromState_ <<
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0;
    constexpr double T = SAMPLING_PERIOD;
    double S = T * T / 2; // "square"
    double C = T * T * T / 6; // "cube"
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> stateMatrix;
    stateMatrix <<
      1, 0, T, 0, S, 0,
      0, 1, 0, T, 0, S,
      0, 0, 1, 0, T, 0,
      0, 0, 0, 1, 0, T,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, STATE_SIZE, INPUT_SIZE> inputMatrix;
    inputMatrix <<
      C, 0,
      0, C,
      S, 0,
      0, S,
      T, 0,
      0, T;
    Eigen::VectorXd biasVector = Eigen::VectorXd::Zero(6);
    initState_ = Eigen::VectorXd::Zero(6);
    previewSystem_ = std::make_shared<copra::PreviewSystem>(
        stateMatrix, inputMatrix, biasVector, initState_, NB_STEPS);
  }

  void HorizontalMPCProblem::configure(const mc_rtc::Configuration & config)
  {
    if (config.has("weights"))
    {
      auto weights = config("weights");
      weights("jerk", jerkWeight);
      weights("vel", velWeights);
      weights("zmp", zmpWeight);
    }
  }

  void HorizontalMPCProblem::phaseDurations(double initSupportDuration, double doubleSupportDuration, double targetSupportDuration)
  {
    constexpr double T = SAMPLING_PERIOD;

    unsigned nbStepsSoFar = 0;
    nbInitSupportSteps_ = std::min(
        static_cast<unsigned>(std::round(initSupportDuration / T)),
        NB_STEPS - nbStepsSoFar);
    nbStepsSoFar += nbInitSupportSteps_;
    nbDoubleSupportSteps_ = std::min(
        static_cast<unsigned>(std::round(doubleSupportDuration / T)),
        NB_STEPS - nbStepsSoFar);
    nbStepsSoFar += nbDoubleSupportSteps_;
    nbTargetSupportSteps_ = std::min(
        static_cast<unsigned>(std::round(targetSupportDuration / T)),
        NB_STEPS - nbStepsSoFar);
    nbStepsSoFar += nbTargetSupportSteps_;
    if (nbTargetSupportSteps_ > 0) // full preview
    {
      nbNextDoubleSupportSteps_ = NB_STEPS - nbStepsSoFar; // always positive
    }
    for (long i = 0; i <= NB_STEPS; i++)
    {
      // NB: SSP constrained is enforced at the very first step of DSP
      if (i < nbInitSupportSteps_ || (0 < i && i == nbInitSupportSteps_))
      {
        indexToHrep[i] = 0;
      }
      else if (i - nbInitSupportSteps_ < nbDoubleSupportSteps_)
      {
        indexToHrep[i] = 1;
      }
      else if (nbTargetSupportSteps_ > 0)
      {
        if (i - nbInitSupportSteps_ - nbDoubleSupportSteps_ <= nbTargetSupportSteps_)
        {
          indexToHrep[i] = 2;
        }
        else if (nbNextDoubleSupportSteps_ > 0)
        {
          indexToHrep[i] = 3;
        }
        else
        {
          indexToHrep[i] = 2;
        }
      }
      else // (nbTargetSupportSteps_ == 0)
      {
        indexToHrep[i] = 1;
      }
    }
  }

  Eigen::HrepXd HorizontalMPCProblem::getSingleSupportHrep(const Contact & contact)
  {
    Eigen::Matrix<double, 4, 2> contactHrepMat, worldHrepMat;
    Eigen::Matrix<double, 4, 1> contactHrepVec, worldHrepVec;
    contactHrepMat <<
      +1, 0,
      -1, 0,
      0, +1,
      0, -1;
    contactHrepVec <<
      contact.halfLength,
      contact.halfLength,
      contact.halfWidth,
      contact.halfWidth;
    if ((contact.normal() - world::e_z).norm() > 1e-3)
    {
      LOG_ERROR("Contact is not horizontal");
    }
    sva::PTransformd X_0_c = contact.pose;
    worldHrepMat = contactHrepMat * X_0_c.rotation().topLeftCorner<2, 2>();
    worldHrepVec = worldHrepMat * X_0_c.translation().head<2>() + contactHrepVec;
    return Eigen::HrepXd(worldHrepMat, worldHrepVec);
  }

  Eigen::HrepXd HorizontalMPCProblem::getDoubleSupportHrep(const Contact & contact1, const Contact & contact2)
  {
    Eigen::MatrixXd vertices{8, 2};
    Contact contact1n = contact1.addNoise(1e-4);
    Contact contact2n = contact2.addNoise(1e-4);
    vertices <<
      contact1n.vertex0().x(), contact1n.vertex0().y(),
      contact1n.vertex1().x(), contact1n.vertex1().y(),
      contact1n.vertex2().x(), contact1n.vertex2().y(),
      contact1n.vertex3().x(), contact1n.vertex3().y(),
      contact2n.vertex0().x(), contact2n.vertex0().y(),
      contact2n.vertex1().x(), contact2n.vertex1().y(),
      contact2n.vertex2().x(), contact2n.vertex2().y(),
      contact2n.vertex3().x(), contact2n.vertex3().y();
    Eigen::Polyhedron poly;
    if (!poly.setVertices(vertices))
    {
      LOG_WARNING("cddlib conversion error: " << poly.lastErrorMessage());
      return getSingleSupportHrep(contact2);
    }
    return poly.hrep();
  }

  void HorizontalMPCProblem::computeZMPRef()
  {
    velRef_.setZero();
    zmpRef_.setZero();
    Eigen::Vector2d p_0 = initContact_.anklePos().head<2>();
    Eigen::Vector2d p_1 = targetContact_.anklePos().head<2>();
    Eigen::Vector2d p_2 = nextContact_.anklePos().head<2>();
    Eigen::Vector2d v_0 = initContact_.refVel.head<2>();
    Eigen::Vector2d v_1 = targetContact_.refVel.head<2>();
    Eigen::Vector2d v_2 = nextContact_.refVel.head<2>();
    if (nbTargetSupportSteps_ < 1) // stop during first DSP
    {
      p_1 = 0.5 * (initContact_.anklePos() + targetContact_.anklePos()).head<2>();
      v_1 = {0., 0.};
    }
    for (long i = 0; i <= NB_STEPS; i++)
    {
      if (indexToHrep[i] <= 1)
      {
        long i1 = i;
        long j1 = i1 - nbInitSupportSteps_;
        double w = static_cast<double>(i1) / (nbInitSupportSteps_ + nbDoubleSupportSteps_);
        double x = (nbDoubleSupportSteps_ > 0) ? static_cast<double>(j1) / nbDoubleSupportSteps_ : 0.;
        w = clamp(w, 0., 1.);
        x = clamp(x, 0., 1.);
        velRef_.segment<2>(2 * i) = (1. - w) * v_0 + w * v_1;
        zmpRef_.segment<2>(2 * i) = (1. - x) * p_0 + x * p_1;
      }
      else // (indexToHrep[i] <= 3), which implies nbTargetSupportSteps_ > 0
      {
        long i2 = i - nbInitSupportSteps_ - nbDoubleSupportSteps_;
        long j2 = i2 - nbTargetSupportSteps_;
        double w = static_cast<double>(i2) / (nbTargetSupportSteps_ + nbNextDoubleSupportSteps_);
        double x = (nbNextDoubleSupportSteps_ > 0) ? static_cast<double>(j2) / nbNextDoubleSupportSteps_ : 0;
        w = clamp(w, 0., 1.);
        x = clamp(x, 0., 1.);
        velRef_.segment<2>(2 * i) = (1. - w) * v_1 + w * v_2;
        zmpRef_.segment<2>(2 * i) = (1. - x) * p_1 + x * p_2;
      }
    }
  }

  void HorizontalMPCProblem::updateTerminalConstraint()
  {
    Eigen::MatrixXd E_dcm = Eigen::MatrixXd::Zero(2, STATE_SIZE * (NB_STEPS + 1));
    Eigen::MatrixXd E_zmp = Eigen::MatrixXd::Zero(2, STATE_SIZE * (NB_STEPS + 1));
    if (nbTargetSupportSteps_ < 1) // half preview
    {
      unsigned i = nbInitSupportSteps_ + nbDoubleSupportSteps_;
      E_dcm.block<2, 6>(0, 6 * i) = dcmFromState_;
      E_zmp.block<2, 6>(0, 6 * i) = zmpFromState_;
    }
    else // full preview
    {
      E_dcm.rightCols<6>() = dcmFromState_;
      E_zmp.rightCols<6>() = zmpFromState_;
    }
    Eigen::Vector2d dcmTarget = zmpRef_.tail<2>();
    Eigen::Vector2d zmpTarget = zmpRef_.tail<2>();
    termDCMCons_ = std::make_shared<copra::TrajectoryConstraint>(E_dcm, dcmTarget, /* isInequalityConstraint = */ false);
    termZMPCons_ = std::make_shared<copra::TrajectoryConstraint>(E_zmp, zmpTarget, /* isInequalityConstraint = */ false);
  }

  void HorizontalMPCProblem::updateZMPConstraint()
  {
    hreps_[0] = getSingleSupportHrep(initContact_);
    hreps_[1] = getDoubleSupportHrep(initContact_, targetContact_);
    hreps_[2] = getSingleSupportHrep(targetContact_);
    hreps_[3] = getDoubleSupportHrep(targetContact_, nextContact_);
    long totalRows = 0;
    for (long i = 0; i <= NB_STEPS; i++)
    {
      auto hrep = hreps_[indexToHrep[i]];
      totalRows += static_cast<unsigned>(hrep.first.rows());
    }
    Eigen::MatrixXd A{totalRows, STATE_SIZE * (NB_STEPS + 1)};
    Eigen::VectorXd b{totalRows};
    A.setZero();
    long nextRow = 0;
    for (long i = 0; i <= NB_STEPS; i++)
    {
      auto hrep = hreps_[indexToHrep[i]];
      unsigned consRows = static_cast<unsigned>(hrep.first.rows());
      A.block(nextRow, STATE_SIZE * i, consRows, STATE_SIZE) = hrep.first * zmpFromState_;
      b.segment(nextRow, consRows) = hrep.second;
      nextRow += consRows;
    }
    zmpCons_ = std::make_shared<copra::TrajectoryConstraint>(A, b);
  }

  void HorizontalMPCProblem::updateJerkCost()
  {
    Eigen::Matrix2d jerkMat = Eigen::Matrix2d::Identity();
    Eigen::Vector2d jerkVec = Eigen::Vector2d::Zero();
    jerkCost_ = std::make_shared<copra::ControlCost>(jerkMat, jerkVec);
    jerkCost_->weight(jerkWeight);
  }

  void HorizontalMPCProblem::updateVelCost()
  {
    velCost_ = std::make_shared<copra::TrajectoryCost>(velFromState_, velRef_);
    velCost_->weights(velWeights);
    velCost_->autoSpan(); // repeat zmpFromState
  }

  void HorizontalMPCProblem::updateZMPCost()
  {
    zmpCost_ = std::make_shared<copra::TrajectoryCost>(zmpFromState_, zmpRef_);
    zmpCost_->weight(zmpWeight);
    zmpCost_->autoSpan(); // repeat zmpFromState
  }

  bool HorizontalMPCProblem::solve()
  {
    computeZMPRef();

    previewSystem_->xInit(initState_);
    updateTerminalConstraint();
    updateZMPConstraint();
    updateJerkCost();
    updateVelCost();
    updateZMPCost();

    copra::LMPC lmpc(previewSystem_);
    lmpc.addConstraint(termDCMCons_);
    lmpc.addConstraint(termZMPCons_);
    lmpc.addConstraint(zmpCons_);
    lmpc.addCost(jerkCost_);
    lmpc.addCost(velCost_);
    lmpc.addCost(zmpCost_);

    if (!lmpc.solve())
    {
      LOG_ERROR("Horizontal MPC problem has no solution");
      solution_ = HorizontalMPCSolution(initState_);
      return false;
    }
    else
    {
      solution_ = HorizontalMPCSolution(lmpc.trajectory(), lmpc.control());
      return true;
    }
  }
}
