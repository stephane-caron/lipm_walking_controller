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

#include <iomanip>

#include <lipm_walking/ModelPredictiveControl.h>
#include <lipm_walking/utils/clamp.h>
#include <lipm_walking/utils/slerp.h>

namespace lipm_walking
{
  // Repeat static constexpr declarations
  // Fixes https://github.com/stephane-caron/lipm_walking_controller/issues/21
  // See also https://stackoverflow.com/q/8016780
  constexpr double ModelPredictiveControl::SAMPLING_PERIOD;
  constexpr unsigned ModelPredictiveControl::INPUT_SIZE;
  constexpr unsigned ModelPredictiveControl::NB_STEPS;
  constexpr unsigned ModelPredictiveControl::STATE_SIZE;

  ModelPredictiveControl::ModelPredictiveControl()
  {
    velCostMat_.setZero();
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
    LOG_SUCCESS("Initialized new ModelPredictiveControl solver");
  }

  void ModelPredictiveControl::configure(const mc_rtc::Configuration & config)
  {
    if (config.has("weights"))
    {
      auto weights = config("weights");
      weights("jerk", jerkWeight);
      weights("vel", velWeights);
      weights("zmp", zmpWeight);
    }
  }

  void ModelPredictiveControl::addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
  {
    using namespace mc_rtc::gui;
    gui->addElement(
      {"Walking", "CoM"},
      ArrayInput("MPC cost weights",
        {"jerk", "vel_x", "vel_y", "zmp"},
        [this]()
        {
          Eigen::VectorXd weights(4);
          weights[0] = jerkWeight;
          weights[1] = velWeights.x();
          weights[2] = velWeights.y();
          weights[3] = zmpWeight;
          return weights;
        },
        [this](const Eigen::VectorXd & weights)
        {
          jerkWeight = weights[0];
          velWeights.x() = weights[1];
          velWeights.y() = weights[2];
          zmpWeight = weights[3];
        }),
      ComboInput(
        "MPC QP solver",
        {"QuadProgDense", "QLD", "LSSOL"},
        [this]()
        {
          switch (solver_)
          {
            case copra::SolverFlag::LSSOL:
              return "LSSOL";
            case copra::SolverFlag::QLD:
              return "QLD";
            case copra::SolverFlag::QuadProgDense:
            default:
              return "QuadProgDense";
          }
        },
        [this](const std::string & solver)
        {
          if (solver == "LSSOL")
          {
            solver_ = copra::SolverFlag::LSSOL;
          }
          else if (solver == "QLD")
          {
            solver_ = copra::SolverFlag::QLD;
          }
          else
          {
            solver_ = copra::SolverFlag::QuadProgDense;
          }
        }));
  }

  void ModelPredictiveControl::addLogEntries(mc_rtc::Logger & logger)
  {
    logger.addLogEntry("perf_MPCBuildAndSolve", [this]() { return buildAndSolveTime_; });
    logger.addLogEntry("perf_MPCSolve", [this]() { return solveTime_; });
  }

  void ModelPredictiveControl::phaseDurations(double initSupportDuration, double doubleSupportDuration, double targetSupportDuration)
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
      // SSP constraint is enforced at the very first step of DSP
      if (i < nbInitSupportSteps_ || (0 < i && i == nbInitSupportSteps_))
      {
        indexToHrep_[i] = 0;
      }
      else if (i - nbInitSupportSteps_ < nbDoubleSupportSteps_)
      {
        indexToHrep_[i] = 1;
      }
      else if (nbTargetSupportSteps_ > 0)
      {
        if (i - nbInitSupportSteps_ - nbDoubleSupportSteps_ <= nbTargetSupportSteps_)
        {
          indexToHrep_[i] = 2;
        }
        else if (nbNextDoubleSupportSteps_ > 0)
        {
          indexToHrep_[i] = 3;
        }
        else // (nbNextDoubleSupportSteps_ == 0)
        {
          indexToHrep_[i] = 2;
        }
      }
      else // (nbTargetSupportSteps_ == 0)
      {
        indexToHrep_[i] = 1;
      }
    }
  }

  void ModelPredictiveControl::computeZMPRef()
  {
    zmpRef_.setZero();
    Eigen::Vector2d p_0 = initContact_.anklePos().head<2>();
    Eigen::Vector2d p_1 = targetContact_.anklePos().head<2>();
    Eigen::Vector2d p_2 = nextContact_.anklePos().head<2>();
    if (nbTargetSupportSteps_ < 1) // stop during first DSP
    {
      p_1 = 0.5 * (initContact_.anklePos() + targetContact_.anklePos()).head<2>();
    }
    for (long i = 0; i <= NB_STEPS; i++)
    {
      if (indexToHrep_[i] <= 1)
      {
        long j = i - nbInitSupportSteps_;
        double x = (nbDoubleSupportSteps_ > 0) ? static_cast<double>(j) / nbDoubleSupportSteps_ : 0.;
        x = clamp(x, 0., 1.);
        zmpRef_.segment<2>(2 * i) = (1. - x) * p_0 + x * p_1;
      }
      else // (indexToHrep_[i] <= 3), which implies nbTargetSupportSteps_ > 0
      {
        long j = i - nbInitSupportSteps_ - nbDoubleSupportSteps_ - nbTargetSupportSteps_;
        double x = (nbNextDoubleSupportSteps_ > 0) ? static_cast<double>(j) / nbNextDoubleSupportSteps_ : 0;
        x = clamp(x, 0., 1.);
        zmpRef_.segment<2>(2 * i) = (1. - x) * p_1 + x * p_2;
      }
    }
  }

  void ModelPredictiveControl::updateTerminalConstraint()
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

  void ModelPredictiveControl::updateZMPConstraint()
  {
    hreps_[0] = initContact_.hrep();
    hreps_[2] = targetContact_.hrep();
    unsigned totalRows = 0;
    for (long i = 0; i <= NB_STEPS; i++)
    {
      unsigned hrepIndex = indexToHrep_[i];
      if (hrepIndex % 2 == 0)
      {
        const auto & hrep = hreps_[hrepIndex];
        totalRows += static_cast<unsigned>(hrep.first.rows());
      }
    }
    Eigen::MatrixXd A{totalRows, STATE_SIZE * (NB_STEPS + 1)};
    Eigen::VectorXd b{totalRows};
    A.setZero();
    long nextRow = 0;
    for (long i = 0; i <= NB_STEPS; i++)
    {
      unsigned hrepIndex = indexToHrep_[i];
      if (hrepIndex % 2 == 0)
      {
        const auto & hrep = hreps_[indexToHrep_[i]];
        unsigned consRows = static_cast<unsigned>(hrep.first.rows());
        A.block(nextRow, STATE_SIZE * i, consRows, STATE_SIZE) = hrep.first * zmpFromState_;
        b.segment(nextRow, consRows) = hrep.second;
        nextRow += consRows;
      }
    }
    zmpCons_ = std::make_shared<copra::TrajectoryConstraint>(A, b);
  }

  void ModelPredictiveControl::updateJerkCost()
  {
    Eigen::Matrix2d jerkMat = Eigen::Matrix2d::Identity();
    Eigen::Vector2d jerkVec = Eigen::Vector2d::Zero();
    jerkCost_ = std::make_shared<copra::ControlCost>(jerkMat, jerkVec);
    jerkCost_->weight(jerkWeight);
  }

  void ModelPredictiveControl::updateVelCost()
  {
    velRef_.setZero();
    const Eigen::Matrix3d & R_0 = initContact_.pose.rotation();
    const Eigen::Matrix3d & R_1 = targetContact_.pose.rotation();
    const Eigen::Matrix3d & R_2 = nextContact_.pose.rotation();
    Eigen::Vector2d v_0 = initContact_.refVel.head<2>();
    Eigen::Vector2d v_1 = targetContact_.refVel.head<2>();
    Eigen::Vector2d v_2 = nextContact_.refVel.head<2>();
    if (nbTargetSupportSteps_ < 1) // stop during first DSP
    {
      v_1 = {0., 0.};
    }
    Eigen::Matrix2d R;
    Eigen::Vector2d v;
    for (long i = 0; i <= NB_STEPS; i++)
    {
      if (indexToHrep_[i] <= 1)
      {
        double w = static_cast<double>(i) / (nbInitSupportSteps_ + nbDoubleSupportSteps_);
        w = clamp(w, 0., 1.);
        R = slerp(R_0, R_1, w).topLeftCorner<2, 2>();
        v = (1. - w) * v_0 + w * v_1;
      }
      else // (indexToHrep_[i] <= 3), which implies nbTargetSupportSteps_ > 0
      {
        long i2 = i - nbInitSupportSteps_ - nbDoubleSupportSteps_; // >= 0
        double w = static_cast<double>(i2) / (nbTargetSupportSteps_ + nbNextDoubleSupportSteps_);
        w = clamp(w, 0., 1.);
        R = slerp(R_1, R_2, w).topLeftCorner<2, 2>();;
        v = (1. - w) * v_1 + w * v_2;
      }
      velCostMat_.block<2, STATE_SIZE>(2 * i, STATE_SIZE * i).block<2, 2>(0, 2) = R;
      velRef_.segment<2>(2 * i) = R * v;
    }
    velCost_ = std::make_shared<copra::TrajectoryCost>(velCostMat_, velRef_);
    velCost_->weights(velWeights);
  }

  void ModelPredictiveControl::updateZMPCost()
  {
    zmpCost_ = std::make_shared<copra::TrajectoryCost>(zmpFromState_, zmpRef_);
    zmpCost_->weight(zmpWeight);
    zmpCost_->autoSpan(); // repeat zmpFromState
  }

  bool ModelPredictiveControl::solve()
  {
    using namespace std::chrono;
    auto startTime = high_resolution_clock::now();

    computeZMPRef();

    previewSystem_->xInit(initState_);
    updateTerminalConstraint();
    updateZMPConstraint();
    updateJerkCost();
    updateVelCost();
    updateZMPCost();

    copra::LMPC lmpc(previewSystem_, solver_);
    lmpc.addConstraint(termDCMCons_);
    lmpc.addConstraint(termZMPCons_);
    lmpc.addConstraint(zmpCons_);
    lmpc.addCost(jerkCost_);
    lmpc.addCost(velCost_);
    lmpc.addCost(zmpCost_);

    bool solutionFound = lmpc.solve();
    if (solutionFound)
    {
      solution_.reset(new Preview(lmpc.trajectory(), lmpc.control()));
    }
    else
    {
      LOG_ERROR("Model predictive control problem has no solution");
      solution_.reset(new Preview());
    }

    auto endTime = high_resolution_clock::now();
    buildAndSolveTime_ = 1000. * duration_cast<duration<double>>(endTime - startTime).count();
    solveTime_ = 1000. * lmpc.solveTime();
    return solutionFound;
  }
}
