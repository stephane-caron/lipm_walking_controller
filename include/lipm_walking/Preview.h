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

#include <lipm_walking/Pendulum.h>

namespace lipm_walking
{
  /** Solution of a model predictive control problem.
   *
   * \note A Preview is mostly an integrator. Its implementation is coupled
   * with the formulation of state and input trajectories in the
   * ModelPredictiveControl class.
   *
   */
  struct Preview
  {
    /** Initialize with zero state and input trajectories.
     *
     */
    Preview();

    /** Initialize solution from trajectories.
     *
     * \param stateTraj State trajectory.
     *
     * \param inputTraj Input trajectory.
     *
     */
    Preview(const Eigen::VectorXd & stateTraj, const Eigen::VectorXd & inputTraj);

    /** Integrate playback on reference.
     *
     * \param state State to integrate.
     *
     * \param dt Duration.
     *
     */
    void integrate(Pendulum & state, double dt);

    /** Playback integration of state reference.
     *
     * \param state State to integrate.
     *
     * \param dt Duration.
     *
     */
    void integratePlayback(Pendulum & state, double dt);

    /** Post-playback integration of state reference.
     *
     * \param state State to integrate.
     *
     * \param dt Duration.
     *
     */
    void integratePostPlayback(Pendulum & state, double dt);

    /** Get current playback step.
     *
     */
    unsigned playbackStep() const
    {
      return playbackStep_;
    }

    /** Get current playback time.
     *
     */
    double playbackTime() const
    {
      return playbackTime_;
    }

  private:
    Eigen::VectorXd inputTraj_; /**< Stacked vector of input trajectory */
    Eigen::VectorXd stateTraj_; /**< Stacked vector of state trajectory */
    double playbackTime_ = 0.; /**< Current time in the preview window */
    unsigned playbackStep_ = 0; /**< Current step index in the preview window */
  };
}
