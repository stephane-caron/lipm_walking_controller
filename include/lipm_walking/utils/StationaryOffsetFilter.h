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
