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

#include <fstream>

#include <lipm_walking/HorizontalMPCSolution.h>

namespace lipm_walking
{
  using namespace HorizontalMPC;

  HorizontalMPCSolution::HorizontalMPCSolution(const Eigen::VectorXd & initState)
  {
    jerkTraj_ = Eigen::VectorXd::Zero(NB_STEPS * INPUT_SIZE);
    stateTraj_ = Eigen::VectorXd::Zero((NB_STEPS + 1) * STATE_SIZE);
    stateTraj_.head<STATE_SIZE>() = initState;
  }

  HorizontalMPCSolution::HorizontalMPCSolution(const Eigen::VectorXd & stateTraj, const Eigen::VectorXd & jerkTraj)
  {
    if (stateTraj.size() / STATE_SIZE != 1 + jerkTraj.size() / INPUT_SIZE)
    {
      LOG_ERROR("Invalid state/input sizes, respectively " << stateTraj.size() << " and " << jerkTraj.size());
    }
    jerkTraj_ = jerkTraj;
    stateTraj_ = stateTraj;
  }

  void HorizontalMPCSolution::integrate(Pendulum & pendulum, double dt)
  {
    if (playbackStep_ < NB_STEPS)
    {
      integratePlayback(pendulum, dt);
    }
    else // (playbackStep_ >= NB_STEPS)
    {
      integratePostPlayback(pendulum, dt);
    }
  }

  void HorizontalMPCSolution::integratePlayback(Pendulum & pendulum, double dt)
  {
    Eigen::Vector3d comddd;
    comddd.head<INPUT_SIZE>() = jerkTraj_.segment<INPUT_SIZE>(INPUT_SIZE * playbackStep_);
    comddd.z() = 0.;
    playbackTime_ += dt;
    if (playbackTime_ >= (playbackStep_ + 1) * SAMPLING_PERIOD)
    {
      playbackStep_++;
    }
    pendulum.integrateCoMJerk(comddd, dt);
  }

  void HorizontalMPCSolution::integratePostPlayback(Pendulum & pendulum, double dt)
  {
    Eigen::Vector3d comddd;
    Eigen::VectorXd lastState = stateTraj_.segment<STATE_SIZE>(STATE_SIZE * NB_STEPS);
    Eigen::Vector2d comd_f = lastState.segment<2>(2);
    Eigen::Vector2d comdd_f = lastState.segment<2>(4);
    if (std::abs(comd_f.x() * comdd_f.y() - comd_f.y() * comdd_f.x()) > 1e-4)
    {
      LOG_WARNING("HMPC terminal condition is not properly fulfilled");
    }
    double omega_f = -comd_f.dot(comdd_f) / comd_f.dot(comd_f);
    double lambda_f = std::pow(omega_f, 2);
    comddd = -omega_f * pendulum.comdd() - lambda_f * pendulum.comd();
    comddd.z() = 0.;
    pendulum.integrateCoMJerk(comddd, dt);
  }
}
