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

/** Integrator
 *
 * The output y(t) of this integrator w.r.t. its input x(t) follows
 *
 *    yd(t) = x(t) - decay * y(t)
 *
 * so that:
 *
 *    y(t) = int_{u=0}^t x(u) e^{decay * (u - t)} d{u}
 *
 * where decay > 0 is a reset frequency used for anti-windup.
 *
 */
struct Integrator
{
  /** Reset integral to zero.
   *
   */
  void reset()
  {
    integral_.setZero();
  }

  /** Add constant input for a fixed duration.
   *
   * \param value Constant input.
   *
   * \param dt Fixed duration.
   *
   */
  void add(const Eigen::Vector3d & value, double dt)
  {
    integral_ = (1. - decay_ * dt) * integral_ + dt * value;
    if (saturation_ > 0.)
    {
      saturate();
    }
  }

  /** Get decay frequency.
   *
   */
  double decay() const
  {
    return decay_;
  }

  /** Set decay frequency.
   *
   * \param decay New frequency.
   *
   */
  void decay(double decay)
  {
    decay_ = decay;
  }

  /** Evaluate the output of the integrator.
   *
   */
  const Eigen::Vector3d & eval() const
  {
    return integral_;
  }

  /** Set output saturation. Disable by providing a negative value.
   *
   * \param s Output will saturate between -s and +s.
   *
   */
  void saturation(double s)
  {
    saturation_ = s;
  }

private:
  void saturate()
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
  double decay_ = 0.1;
  double saturation_ = -1.;
};
