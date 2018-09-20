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

namespace lipm_walking
{
  /** Low-pass velocity filter from series of position measurements.
   *
   */
  template <typename T>
  struct VelocityFilter
  {
    /** Constructor.
     *
     * \param dt Sampling period.
     *
     */
    VelocityFilter(double dt)
      : dt_(dt)
    {
      reset(T::Zero());
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

    /** Read settings from configuration file.
     *
     * \param config Configuration dictionary.
     *
     */
    void configure(const mc_rtc::Configuration & config)
    {
      config("cutoff_period", cutoffPeriod_);
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
      cutoffPeriod_ = period;
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
