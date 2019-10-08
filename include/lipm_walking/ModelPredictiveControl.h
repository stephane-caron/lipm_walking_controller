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

#include <copra/constraints.h>
#include <copra/costFunctions.h>
#include <copra/LMPC.h>
#include <copra/PreviewSystem.h>

#include <lipm_walking/Contact.h>
#include <lipm_walking/Pendulum.h>
#include <lipm_walking/Preview.h>
#include <lipm_walking/defs.h>

namespace lipm_walking
{
  /** Solution to a model predictive control problem.
   *
   */
  struct ModelPredictiveControlSolution : Preview
  {
    /** Initialize a zero solution with a given initial state.
     *
     * \param initState Initial state.
     *
     */
    ModelPredictiveControlSolution(const Eigen::VectorXd & initState);

    /** Initialize solution from trajectories.
     *
     * \param stateTraj State trajectory.
     *
     * \param jerkTraj CoM jerk trajectory.
     *
     */
    ModelPredictiveControlSolution(const Eigen::VectorXd & stateTraj, const Eigen::VectorXd & jerkTraj);

    /** Integrate playback on reference.
     *
     * \param state CoM state to integrate upon.
     *
     * \param dt Duration.
     *
     */
    void integrate(Pendulum & state, double dt);

    /** Playback integration of CoM state reference.
     *
     * \param state CoM state to integrate upon.
     *
     * \param dt Duration.
     *
     */
    void integratePlayback(Pendulum & state, double dt);

    /** Post-playback integration of CoM state reference.
     *
     * \param state CoM state to integrate upon.
     *
     * \param dt Duration.
     *
     */
    void integratePostPlayback(Pendulum & state, double dt);

    /** Fill solution with zeros, except for initial state.
     *
     * \param initState Initial state.
     *
     */
    void zeroFrom(const Eigen::VectorXd & initState);

    /** Get the CoM jerk (input) trajectory.
     *
     */
    const Eigen::VectorXd & jerkTraj() const
    {
      return jerkTraj_;
    }

    /** Get the CoM state trajectory.
     *
     */
    const Eigen::VectorXd & stateTraj() const
    {
      return stateTraj_;
    }

  private:
    Eigen::VectorXd jerkTraj_; /**< Stacked vector of CoM jerk trajectory */
    Eigen::VectorXd stateTraj_; /**< Stacked vector of CoM state trajectory */
  };

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
      zeta_ = height / world::GRAVITY;
      double omegaInv = std::sqrt(zeta_);
      dcmFromState_ << 
        1, 0, omegaInv, 0, 0, 0,
        0, 1, 0, omegaInv, 0, 0;
      zmpFromState_ <<
        1, 0, 0, 0, -zeta_, 0,
        0, 1, 0, 0, 0, -zeta_;
    }

    /** Reset contacts.
     *
     * \param initContact Contact used during single-support phase.
     *
     * \param targetContact Contact used during double-support phases.
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
      initState_ = Eigen::VectorXd(6);
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

    using RefVec = Eigen::Matrix<double, 2 * (NB_STEPS + 1), 1>;

    const RefVec & velRef() const
    {
      return velRef_;
    }

    double zeta() const
    {
      return zeta_;
    }

    const RefVec & zmpRef() const
    {
      return zmpRef_;
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
    Contact initContact_;
    Contact nextContact_;
    Contact targetContact_;
    Eigen::HrepXd hreps_[4]; /**< ZMP inequality constraints (H-rep) */
    Eigen::Matrix<double, 2 * (NB_STEPS + 1), 1> velRef_;
    Eigen::Matrix<double, 2 * (NB_STEPS + 1), 1> zmpRef_;
    Eigen::Matrix<double, 2 * (NB_STEPS + 1), STATE_SIZE * (NB_STEPS + 1)> velCostMat_;
    Eigen::Matrix<double, 2, STATE_SIZE> dcmFromState_;
    Eigen::Matrix<double, 2, STATE_SIZE> zmpFromState_;
    Eigen::VectorXd initState_;
    copra::SolverFlag solver_ = copra::SolverFlag::QLD;
    double buildAndSolveTime_ = 0.; // [s]
    double solveTime_ = 0.; // [s]
    double zeta_;
    std::shared_ptr<ModelPredictiveControlSolution> solution_ = nullptr;
    std::shared_ptr<copra::ControlCost> jerkCost_;
    std::shared_ptr<copra::PreviewSystem> previewSystem_;
    std::shared_ptr<copra::TrajectoryConstraint> termDCMCons_;
    std::shared_ptr<copra::TrajectoryConstraint> termZMPCons_;
    std::shared_ptr<copra::TrajectoryConstraint> zmpCons_;
    std::shared_ptr<copra::TrajectoryCost> velCost_;
    std::shared_ptr<copra::TrajectoryCost> zmpCost_;
    unsigned indexToHrep_[NB_STEPS + 1]; /**< Mapping from timestep index to ZMP inequality constraints */
    unsigned nbDoubleSupportSteps_;
    unsigned nbInitSupportSteps_;
    unsigned nbNextDoubleSupportSteps_;
    unsigned nbTargetSupportSteps_;
  };
}
