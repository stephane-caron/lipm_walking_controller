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

#include <Eigen/Dense>

namespace utils
{
  /** Compute velocity by finite difference of position measurements, applying a
   * low-pass filter to it.
   *
   */
  template <typename T>
  struct LowPassVelocityFilter
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Constructor.
     *
     * \param dt Sampling period.
     *
     */
    LowPassVelocityFilter(double dt)
      : dt_(dt)
    {
      reset(T::Zero());
    }

    /** Constructor with cutoff period.
     *
     * \param dt Sampling period.
     *
     * \param period Cutoff period.
     *
     */
    LowPassVelocityFilter(double dt, double period)
      : dt_(dt)
    {
      reset(T::Zero());
      cutoffPeriod(period);
    }

    /** Get cutoff period.
     *
     */
    double cutoffPeriod() const
    {
      return cutoffPeriod_;
    }

    /** Set cutoff period.
     *
     * \param period New cutoff period.
     *
     */
    void cutoffPeriod(double period)
    {
      period = std::max(period, 2 * dt_);
      cutoffPeriod_ = period;
    }

    /** Reset position to an initial rest value.
     *
     * \param pos New position.
     *
     */
    void reset(T pos)
    {
      pos_ = pos;
      vel_ = T::Zero();
    }

    /** Update velocity estimate from new position value.
     *
     * \param newPos New observed position.
     *
     */
    void update(const T & newPos)
    {
      double x = (cutoffPeriod_ <= dt_) ? 1. : dt_ / cutoffPeriod_;
      T discVel = (newPos - pos_) / dt_;
      T newVel = x * discVel + (1. - x) * vel_;
      pos_ = newPos;
      vel_ = newVel;
    }

    /** Update position only.
     *
     */
    void updatePositionOnly(const T & newPos)
    {
      pos_ = newPos;
    }

    /** Get filtered velocity.
     *
     */
    const T & vel()
    {
      return vel_;
    }

  private:
    T pos_;
    T vel_;
    double cutoffPeriod_ = 0.;
    double dt_ = 0.005; // [s]
  };
}

using utils::LowPassVelocityFilter;
