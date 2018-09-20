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

#include <lipm_walking/PendulumObserver.h>

namespace lipm_walking
{
  PendulumObserver::PendulumObserver(double dt)
    : Pendulum(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      dt_(dt)
  {
  }

  void PendulumObserver::update(const Eigen::Vector3d & comGuess, const sva::ForceVecd & contactWrench, const Contact & contact)
  {
    const Eigen::Vector3d & force = contactWrench.force();
    const Eigen::Vector3d & moment_0 = contactWrench.couple();
    Eigen::Vector3d moment_p = moment_0 - contact.p().cross(force);
    double fSquare = force.dot(force);
    if (fSquare < 42.)
    {
      return;
    }

    Eigen::Vector3d com_prev = com_;
    zmp_ = contact.p() + contact.n().cross(moment_p) / contact.n().dot(force);
    double closestCoMScaling = force.dot(comGuess - zmp_) / fSquare;
    // take the point on the measured central axis that is closest to the guess
    com_ = zmp_ + closestCoMScaling * force;
    comd_ = (com_ - com_prev) / dt_;
    comdd_ = force / mass_;

    double f_n = contact.n().dot(force);
    double lambda = f_n / (mass_ * contact.n().dot(com_ - zmp_));
    omega_ = std::sqrt(lambda);
    if ((contactForce() - force).norm() > 1e-5)
    {
      LOG_ERROR("error in estimated contact force computation");
    }
  }
}
