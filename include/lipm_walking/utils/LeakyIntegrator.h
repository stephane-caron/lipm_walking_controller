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

/** Leaky integrator.
 *
 * The output satisfies the differential equation:
 *
 *     yd(t) = x(t) - leakRate * y(t)
 *
 * A leaky integrator is implemented exactly as an exponential moving average,
 * but it is homogeneous to the integral of the input signal (rather than the
 * signal itself). See <https://en.wikipedia.org/wiki/Leaky_integrator>.
 *
 */
struct LeakyIntegrator
{
  /** Add constant input for a fixed duration.
   *
   * \param value Constant input.
   *
   * \param dt Fixed duration.
   *
   */
  inline void add(const Eigen::Vector3d & value, double dt)
  {
    integral_ = (1. - rate_ * dt) * integral_ + dt * value;
    if (saturation_ > 0.)
    {
      saturate();
    }
  }

  /** Evaluate the output of the integrator.
   *
   */
  inline const Eigen::Vector3d & eval() const
  {
    return integral_;
  }

  /** Get leak rate.
   *
   */
  inline double rate() const
  {
    return rate_;
  }

  /** Set the leak rate of the integrator.
   *
   * \param rate New leak rate.
   *
   */
  inline void rate(double rate)
  {
    rate_ = rate;
  }

  /** Set output saturation. Disable by providing a negative value.
   *
   * \param s Output will saturate between -s and +s.
   *
   */
  inline void saturation(double s)
  {
    saturation_ = s;
  }

  /** Reset integral to zero.
   *
   */
  inline void setZero()
  {
    integral_.setZero();
  }

private:
  inline void saturate()
  {
    for (unsigned i = 0; i < 3; i++)
    {
      if (integral_(i) < -saturation_)
      {
        integral_(i) = -saturation_;
      }
      else if (integral_(i) > saturation_)
      {
        integral_(i) = saturation_;
      }
    }
  }

private:
  Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
  double rate_ = 0.1;
  double saturation_ = -1.;
};
