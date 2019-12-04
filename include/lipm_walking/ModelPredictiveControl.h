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

#include <copra/constraints.h>
#include <copra/costFunctions.h>
#include <copra/LMPC.h>
#include <copra/PreviewSystem.h>

#include <lipm_walking/Contact.h>
#include <lipm_walking/Pendulum.h>
#include <lipm_walking/Preview.h>
#include <lipm_walking/Sole.h>
#include <lipm_walking/utils/world.h>

namespace lipm_walking
{
  /** Model predictive control problem.
   *
   * This implementation is based on "Trajectory free linear model predictive
   * control for stable walking in the presence of strong perturbations"
   * (Wieber, Humanoids 2006) with the addition of terminal constraints.
   *
   */
  struct ModelPredictiveControl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr double SAMPLING_PERIOD = 0.1; // [s]
    static constexpr unsigned INPUT_SIZE = 2; // input is 2D CoM jerk
    static constexpr unsigned NB_STEPS = 16; // number of sampling steps
    static constexpr unsigned STATE_SIZE = 6; // state is CoM [pos, vel, accel]

    using RefVec = Eigen::Matrix<double, 2 * (NB_STEPS + 1), 1>;

    /** Initialize new problem.
     *
     */
    ModelPredictiveControl();

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

    /** Read configuration from dictionary.
     *
     */
    void configure(const mc_rtc::Configuration &);

    /** Set duration of the initial single-support phase.
     *
     * \param initSupportDuration First SSP duration.
     *
     * \param doubleSupportDuration First DSP duration.
     *
     * \param targetSupportDuration Second SSP duration.
     *
     * Phase durations don't have to sum up to the total duration of the
     * preview horizon.
     *
     * If their sum is below total duration, there are two outcomes: if there
     * is a target support phase, a second DSP phase is added from the target
     * contact to the next (full preview mode); otherwise, the first DSP phase
     * is extended until the end of the preview horizon (half preview mode).
     *
     * If their sum exceeds total duration, phase durations are trimmed
     * starting from the last one.
     */
    void phaseDurations(double initSupportDuration, double doubleSupportDuration, double targetSupportDuration);

    /** Solve the model predictive control problem.
     *
     * \returns solutionFound Did the solver find a solution?
     *
     */
    bool solve();

    /** Set CoM height.
     *
     * \param height CoM height above contacts.
     *
     */
    void comHeight(double height)
    {
      double zeta = height / world::GRAVITY;
      double omegaInv = std::sqrt(zeta);
      dcmFromState_ <<
        1, 0, omegaInv, 0, 0, 0,
        0, 1, 0, omegaInv, 0, 0;
      zmpFromState_ <<
        1, 0, 0, 0, -zeta, 0,
        0, 1, 0, 0, 0, -zeta;
    }

    /** Reset contacts.
     *
     * \param initContact Contact used during single-support phase.
     *
     * \param targetContact Contact used during double-support phases.
     *
     * \param nextContact Contact coming after targetContact in the plan.
     *
     */
    void contacts(Contact initContact, Contact targetContact, Contact nextContact)
    {
      initContact_ = initContact;
      nextContact_ = nextContact;
      targetContact_ = targetContact;
    }

    /** Set the initial CoM state.
     *
     * \param pendulum CoM state.
     *
     */
    void initState(const Pendulum & pendulum)
    {
      initState_ = Eigen::VectorXd(STATE_SIZE);
      initState_ <<
        pendulum.com().head<2>(),
        pendulum.comd().head<2>(),
        pendulum.comdd().head<2>();
    }

    /** Get solution vector.
     *
     */
    std::shared_ptr<Preview> solution()
    {
      return solution_;
    }

    /** Get index of inequality constraints
     *
     * \param i Timestep in preview horizon.
     *
     * \returns hrepIndex Index of corresponding ZMP H-rep.
     *
     * \note Inequality constraints are the halfspace representation (H-rep) of
     * a polyhedron. In this class we call them H-rep for short.
     *
     */
    unsigned indexToHrep(unsigned i) const
    {
      return indexToHrep_[i];
    }

    unsigned nbInitSupportSteps() const
    {
      return nbInitSupportSteps_;
    }

    unsigned nbDoubleSupportSteps() const
    {
      return nbDoubleSupportSteps_;
    }

    std::string phaseLabel() const
    {
      std::stringstream label;
      label
        << "ss" << nbInitSupportSteps_ << "-"
        << "ds" << nbDoubleSupportSteps_ << "-"
        << "ts" << nbTargetSupportSteps_ << "-"
        << "nds" << nbNextDoubleSupportSteps_;
      return label.str();
    }

    const Contact & initContact() const
    {
      return initContact_;
    }

    const Contact & targetContact() const
    {
      return targetContact_;
    }

    const Contact & nextContact() const
    {
      return nextContact_;
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

  private:
    void computeZMPRef();

    void updateTerminalConstraint();

    void updateZMPConstraint();

    void updateJerkCost();

    void updateVelCost();

    void updateZMPCost();

  public:
    Eigen::Vector2d velWeights = {10., 10.}; /**< Weights of CoM velocity tracking cost */
    double jerkWeight = 1.; /**< Weight of CoM jerk regularization cost */
    double zmpWeight = 1000.; /**< Weight of reference ZMP tracking cost */

  private:
    Contact initContact_; /**< First support contact */
    Contact nextContact_; /**< Third (optional) support contact */
    Contact targetContact_; /**< Second support contact */
    Eigen::Matrix<double, 2 * (NB_STEPS + 1), 1> velRef_; /**< Stacked vector of reference CoM velocities */
    Eigen::Matrix<double, 2 * (NB_STEPS + 1), 1> zmpRef_; /**< Stacked vector of reference ZMPs */
    Eigen::Matrix<double, 2 * (NB_STEPS + 1), STATE_SIZE * (NB_STEPS + 1)> velCostMat_;
    Eigen::Matrix<double, 2, STATE_SIZE> dcmFromState_; /**< Linear map to extract the DCM of a CoM state (position, velocity, acceleration) */
    Eigen::Matrix<double, 2, STATE_SIZE> zmpFromState_; /**< Linear map to compute the ZMP of a CoM state (position, velocity, acceleration) */
    Eigen::VectorXd initState_; /**< Initial CoM state (position, velocity, acceleration) */
    HrepXd hreps_[4]; /**< Halfspace representation for ZMP inequality constraints */
    copra::SolverFlag solver_ = copra::SolverFlag::QLD; /**< Quadratic programming solver */
    double buildAndSolveTime_ = 0.; /**< Time in [s] taken to build and solve the MPC problem */
    double solveTime_ = 0.; /**< Time in [s] taken to solve the MPC problem */
    std::shared_ptr<Preview> solution_ = nullptr;
    std::shared_ptr<copra::ControlCost> jerkCost_;
    std::shared_ptr<copra::PreviewSystem> previewSystem_;
    std::shared_ptr<copra::TrajectoryConstraint> termDCMCons_;
    std::shared_ptr<copra::TrajectoryConstraint> termZMPCons_;
    std::shared_ptr<copra::TrajectoryConstraint> zmpCons_;
    std::shared_ptr<copra::TrajectoryCost> velCost_;
    std::shared_ptr<copra::TrajectoryCost> zmpCost_;
    unsigned indexToHrep_[NB_STEPS + 1]; /**< Mapping from timestep index to ZMP inequality constraints */
    unsigned nbDoubleSupportSteps_; /**< Number of discretization steps for the double support phase */
    Sole sole_; /**< Sole dimensions of the robot model */
    unsigned nbInitSupportSteps_;
    unsigned nbNextDoubleSupportSteps_;
    unsigned nbTargetSupportSteps_;
  };
}
