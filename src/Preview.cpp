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
#include <lipm_walking/Preview.h>
#include <lipm_walking/utils/clamp.h>
#include <lipm_walking/utils/slerp.h>

namespace lipm_walking
{

namespace
{

constexpr double SAMPLING_PERIOD = ModelPredictiveControl::SAMPLING_PERIOD;
constexpr unsigned INPUT_SIZE = ModelPredictiveControl::INPUT_SIZE;
constexpr unsigned NB_STEPS = ModelPredictiveControl::NB_STEPS;
constexpr unsigned STATE_SIZE = ModelPredictiveControl::STATE_SIZE;

} // namespace

Preview::Preview()
{
  inputTraj_ = Eigen::VectorXd::Zero(NB_STEPS * INPUT_SIZE);
  stateTraj_ = Eigen::VectorXd::Zero((NB_STEPS + 1) * STATE_SIZE);
}

Preview::Preview(const Eigen::VectorXd & stateTraj, const Eigen::VectorXd & inputTraj)
{
  if(stateTraj.size() / STATE_SIZE != 1 + inputTraj.size() / INPUT_SIZE)
  {
    LOG_ERROR("Invalid state/input sizes, respectively " << stateTraj.size() << " and " << inputTraj.size());
  }
  stateTraj_ = stateTraj;
  inputTraj_ = inputTraj;
}

void Preview::integrate(Pendulum & pendulum, double dt)
{
  if(playbackStep_ < NB_STEPS)
  {
    integratePlayback(pendulum, dt);
  }
  else // (playbackStep_ >= NB_STEPS)
  {
    integratePostPlayback(pendulum, dt);
  }
}

void Preview::integratePlayback(Pendulum & pendulum, double dt)
{
  Eigen::Vector3d comddd;
  comddd.head<INPUT_SIZE>() = inputTraj_.segment<INPUT_SIZE>(INPUT_SIZE * playbackStep_);
  comddd.z() = 0.;
  playbackTime_ += dt;
  if(playbackTime_ >= (playbackStep_ + 1) * SAMPLING_PERIOD)
  {
    playbackStep_++;
  }
  pendulum.integrateCoMJerk(comddd, dt);
}

void Preview::integratePostPlayback(Pendulum & pendulum, double dt)
{
  Eigen::Vector3d comddd;
  Eigen::VectorXd lastState = stateTraj_.segment<STATE_SIZE>(STATE_SIZE * NB_STEPS);
  Eigen::Vector2d comd_f = lastState.segment<2>(2);
  Eigen::Vector2d comdd_f = lastState.segment<2>(4);
  if(std::abs(comd_f.x() * comdd_f.y() - comd_f.y() * comdd_f.x()) > 1e-4)
  {
    LOG_WARNING("MPC terminal condition is not properly fulfilled");
  }
  double omega_f = -comd_f.dot(comdd_f) / comd_f.dot(comd_f);
  double lambda_f = std::pow(omega_f, 2);
  comddd = -omega_f * pendulum.comdd() - lambda_f * pendulum.comd();
  comddd.z() = 0.;
  pendulum.integrateCoMJerk(comddd, dt);
}

} // namespace lipm_walking
