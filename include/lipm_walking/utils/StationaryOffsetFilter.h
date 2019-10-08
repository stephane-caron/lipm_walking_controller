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

#include "ExponentialMovingAverage.h"

/** Remove stationary offset from an input signal.
 *
 */
struct StationaryOffsetFilter
{
  /** Constructor.
   *
   * \param dt Time in [s] between two readings.
   *
   * \param timeConstant Length of recent-past window used to evaluate the
   * stationary offset.
   *
   * \param initValue Initial value of the input signal.
   *
   */
  StationaryOffsetFilter(double dt, double timeConstant, const Eigen::Vector3d & initValue = Eigen::Vector3d::Zero())
    : average_(dt, timeConstant, initValue)
  {
    filteredValue_ = initValue;
    rawValue_ = initValue;
  }

  /** Update input signal value.
   *
   * \param value New value.
   *
   */
  void update(const Eigen::Vector3d & value)
  {
    average_.append(value);
    filteredValue_ = value - average_.eval();
    rawValue_ = value;
  }

  /** Get output value where the stationary offset has been filtered.
   *
   */
  const Eigen::Vector3d & eval() const
  {
    return filteredValue_;
  }

  /** Get raw value of input signal.
   *
   */
  const Eigen::Vector3d & raw() const
  {
    return rawValue_;
  }

  /** Reset everything to zero.
   *
   */
  void setZero()
  {
    average_.setZero();
    filteredValue_.setZero();
    rawValue_.setZero();
  }

  /** Get time constant of the filter.
   *
   */
  double timeConstant() const
  {
    return average_.timeConstant();
  }

  /** Update time constant.
   *
   * \param T New time constant of the filter.
   *
   */
  void timeConstant(double T)
  {
    average_.timeConstant(T);
  }

private:
  Eigen::Vector3d filteredValue_;
  Eigen::Vector3d rawValue_;
  ExponentialMovingAverage average_;
};
