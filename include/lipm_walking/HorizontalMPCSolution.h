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

#include <lipm_walking/Pendulum.h>
#include <lipm_walking/Contact.h>
#include <lipm_walking/HorizontalMPC.h>

namespace lipm_walking
{
  /** MPC solution with integrator for playback.
   *
   */
  struct HorizontalMPCSolution
  {
    /** Default constructor.
     *
     */
    HorizontalMPCSolution() = default;

    /** Initialize a zero solution with a given initial state.
     *
     * \param initState Initial state.
     *
     */
    HorizontalMPCSolution(const Eigen::VectorXd & initState);

    /** Default copy constructor.
     *
     */
    HorizontalMPCSolution(const HorizontalMPCSolution &) = default;

    /** Initialize solution from trajectories.
     *
     * \param stateTraj State trajectory.
     *
     * \param jerkTraj CoM jerk trajectory.
     *
     */
    HorizontalMPCSolution(const Eigen::VectorXd & stateTraj, const Eigen::VectorXd & jerkTraj);

    /** Fill solution with zeros, except for initial state.
     *
     * \param initState Initial state.
     *
     */
    void zeroFrom(const Eigen::VectorXd & initState);

    /** Integrate playback on reference.
     *
     * \param state CoM state to integrate upon.
     *
     * \param dt Duration.
     *
     * \note This function only applies to one-step capture solutions.
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

    /** Get the CoM state trajectory.
     *
     */
    const Eigen::VectorXd & stateTraj()
    {
      return stateTraj_;
    }

    /** Get the CoM jerk (input) trajectory.
     *
     */
    const Eigen::VectorXd & jerkTraj()
    {
      return jerkTraj_;
    }

    /** Get current playback step.
     *
     */
    inline unsigned playbackStep()
    {
      return playbackStep_;
    }

    /** Get current playback time.
     *
     */
    inline double playbackTime()
    {
      return playbackTime_;
    }

  private:
    Eigen::VectorXd jerkTraj_;
    Eigen::VectorXd stateTraj_;
    double playbackTime_ = 0.;
    unsigned playbackStep_ = 0;
  };
}
