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

#pragma once

#include <copra/constraints.h>
#include <copra/costFunctions.h>
#include <copra/LMPC.h>
#include <copra/PreviewSystem.h>
#include <eigen-cddlib/Polyhedron.h>

#include <lipm_walking/Contact.h>
#include <lipm_walking/HorizontalMPC.h>
#include <lipm_walking/HorizontalMPCSolution.h>
#include <lipm_walking/defs.h>

namespace lipm_walking
{
  /** Model Predictive Control problem for horizontal walking.
   *
   * This implementation is based on "Trajectory free linear model predictive
   * control for stable walking in the presence of strong perturbations"
   * (Wieber, Humanoids 2006) with the addition of terminal constraints.
   *
   */
  struct HorizontalMPCProblem
  {
    /** Initialize a new capture problem.
     *
     */
    HorizontalMPCProblem();

    /** Read configuration from dictionary.
     *
     */
    void configure(const mc_rtc::Configuration &);

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
     * \param state CoM state.
     *
     */
    void initState(const Pendulum & state)
    {
      initState_ = Eigen::VectorXd(6);
      initState_ << 
        state.com().head<2>(), 
        state.comd().head<2>(),
        state.comdd().head<2>();
    }

    /** Get solution vector.
     *
     */
    const HorizontalMPCSolution & solution()
    {
      return solution_;
    }

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

    /** Set the target CoM height.
     *
     */
    void comHeight(double height)
    {
      comHeight_ = height;
      zeta_ = height / world::GRAVITY;
      double omegaInv = std::sqrt(zeta_);
      dcmFromState_ << 
        1, 0, omegaInv, 0, 0, 0,
        0, 1, 0, omegaInv, 0, 0;
      zmpFromState_ <<
        1, 0, 0, 0, -zeta_, 0,
        0, 1, 0, 0, 0, -zeta_;
    }

    /** Solve the capture problem with an external optimization over alpha.
     *
     * \returns solutionFound Did the solver find a solution?
     *
     */
    bool solve();

    /** Write problem and solution to Python script.
     *
     * \param suffix File name suffix.
     *
     */
    void writePython(const std::string & suffix = "");

  private:
    Eigen::HrepXd getDoubleSupportHrep(const Contact & contact1, const Contact & contact2);

    Eigen::HrepXd getSingleSupportHrep(const Contact & contact);

    void computeZMPRef();

    void updateTerminalConstraint();

    void updateZMPConstraint();

    void updateJerkCost();

    void updateVelCost();

    void updateZMPCost();

    void writePythonContact(const Contact & contact, const std::string & label);

    void writePythonSerializedVector(const Eigen::VectorXd & vec, const std::string & label, unsigned index, unsigned nbChunks);

    void writePythonSolution();

  public:
    Eigen::Vector2d velWeights = {10., 10.};
    double jerkWeight = 1.;
    double zmpWeight = 1000.;

  private:
    Contact initContact_;
    Contact nextContact_;
    Contact targetContact_;
    Eigen::HrepXd hreps_[4];
    Eigen::Matrix<double, 2 * (HorizontalMPC::NB_STEPS + 1), 1> velRef_;
    Eigen::Matrix<double, 2 * (HorizontalMPC::NB_STEPS + 1), 1> zmpRef_;
    Eigen::Matrix<double, 2, HorizontalMPC::STATE_SIZE> dcmFromState_;
    Eigen::Matrix<double, 2, HorizontalMPC::STATE_SIZE> velFromState_;
    Eigen::Matrix<double, 2, HorizontalMPC::STATE_SIZE> zmpFromState_;
    Eigen::Polyhedron cdd_;
    Eigen::VectorXd initState_;
    HorizontalMPCSolution solution_;
    double comHeight_;
    double zeta_;
    std::ofstream pyScript_;
    std::shared_ptr<copra::ControlCost> jerkCost_;
    std::shared_ptr<copra::PreviewSystem> previewSystem_;
    std::shared_ptr<copra::TrajectoryConstraint> termDCMCons_;
    std::shared_ptr<copra::TrajectoryConstraint> termZMPCons_;
    std::shared_ptr<copra::TrajectoryConstraint> zmpCons_;
    std::shared_ptr<copra::TrajectoryCost> velCost_;
    std::shared_ptr<copra::TrajectoryCost> zmpCost_;
    unsigned indexToHrep[HorizontalMPC::NB_STEPS + 1];
    unsigned nbDoubleSupportSteps_;
    unsigned nbInitSupportSteps_;
    unsigned nbNextDoubleSupportSteps_;
    unsigned nbTargetSupportSteps_;
    unsigned nbWritePythonCalls_ = 0;
  };
}
