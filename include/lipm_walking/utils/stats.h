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

/** Average and standard deviation of a time series of scalar values.
 *
 */
struct AvgStdEstimator
{
  /* Add new value to the time series.
   *
   * \param x New value.
   *
   */
  void add(double x)
  {
    n_++;
    total_ += x;
    squareTotal_ += x * x;
    if (x > max_)
    {
      max_ = x;
    }
    if (x < min_)
    {
      min_ = x;
    }
  }

  /** Average of the time series.
   *
   */
  double avg()
  {
    if (n_ < 1)
    {
      return 0.;
    }
    return total_ / n_;
  }

  /** Number of samples.
   *
   */
  unsigned n()
  {
    return n_;
  }

  /** Reset estimator to a empty series.
   *
   */
  void reset()
  {
    max_ = 0.;
    min_ = 0.;
    n_ = 0;
    squareTotal_ = 0.;
    total_ = 0.;
  }

  /** Standard deviation of the time series.
   *
   */
  double std()
  {
    if (n_ <= 1)
    {
      return 0.;
    }
    double unbiased = std::sqrt(double(n_) / (n_ - 1));
    return unbiased * std::sqrt(squareTotal_ / n_ - pow(avg(), 2));
  }

  /** Printout series statistics.
   *
   * \param round Number of output decimals.
   *
   * \param verbose Report extra information such as min and max values.
   *
   */
  std::string str(unsigned round = 0, bool verbose = true)
  {
    std::ostringstream ss;
    double avgVal = avg();
    double stdVal = std();
    double maxVal = max_;
    double minVal = min_;
    if (round > 0)
    {
      double pow = std::pow(10, round);
      avgVal = std::round(pow * avgVal) / pow;
      stdVal = std::round(pow * stdVal) / pow;
      maxVal = std::round(pow * maxVal) / pow;
      minVal = std::round(pow * minVal) / pow;
    }
    ss << avgVal << " +/- " << stdVal;
    if (verbose)
    {
      ss << " (max: " << maxVal << ", min: " << minVal << " over " << n_ << " items)";
    }
    return ss.str();
  }

private:
  double max_ = 0.;
  double min_ = 0.;
  double squareTotal_ = 0.;
  double total_ = 0.;
  unsigned n_ = 0;
};

/** Exponential Moving Average.
 *
 * This filter can be seen as an integrator:
 *
 *    y(t) = 1/T int_{u=0}^t x(u) e^{(u - t) / T} d{u}
 *
 * with T > 0 a reset period acting as anti-windup. It can also (informally) be
 * interpreted as the average value of the input signal x(t) over the last T
 * seconds. Formally, it represents the amount of time for the smoothed
 * response of a unit input to reach 1-1/e (~63%) of the original signal.
 *
 * See <https://en.wikipedia.org/wiki/Exponential_smoothing>. It is equivalent
 * to a low-pass filter <https://en.wikipedia.org/wiki/Low-pass_filter> applied
 * to the integral of the input signal.
 *
 */
struct ExponentialMovingAverage
{
  /** Constructor.
   *
   * \param dt Time in [s] between two readings.
   *
   * \param timeConstant Informally, length of the recent-past window, in [s].
   *
   * \param initValue Initial value of the output average.
   *
   */
  ExponentialMovingAverage(double dt, double timeConstant, const Eigen::Vector3d & initValue = Eigen::Vector3d::Zero())
    : dt_(dt)
  {
    average_ = initValue;
    this->timeConstant(timeConstant);
  }

  /** Append a new reading to the series.
   *
   * \param value New value.
   *
   */
  void append(const Eigen::Vector3d & value)
  {
    average_ += alpha_ * (value - average_);
    if (saturation_ > 0.)
    {
      saturate_();
    }
  }

  /** Evaluate the smoothed statistic.
   *
   */
  const Eigen::Vector3d & eval() const
  {
    return average_;
  }

  /** Set output saturation; disable by providing a negative value.
   *
   * \param limit Output will saturate between -limit and +limit.
   *
   */
  void saturation(double limit)
  {
    saturation_ = limit;
  }

  /** Reset average to zero.
   *
   */
  void setZero()
  {
    average_.setZero();
  }

  /** Get time constant of the filter.
   *
   */
  double timeConstant() const
  {
    return timeConstant_;
  }

  /** Update time constant.
   *
   * \param T New time constant of the filter.
   *
   */
  void timeConstant(double T)
  {
    alpha_ = 1. - std::exp(-dt_ / T);
    //alpha_ = dt_ / T; // valid if dt_ << T
    timeConstant_ = T;
  }

private:
  /** Saturate averaged values.
   *
   */
  void saturate_()
  {
    for (unsigned i = 0; i < 3; i++)
    {
      if (average_(i) < -saturation_)
      {
        average_(i) = -saturation_;
      }
      else if (average_(i) > saturation_)
      {
        average_(i) = saturation_;
      }
    }
  }

protected:
  Eigen::Vector3d average_ = Eigen::Vector3d::Zero();
  double alpha_;
  double dt_;
  double timeConstant_;
  double saturation_ = -1.;
};
