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

/** Online estimator for the average and standard deviation of a time series of
 * scalar values.
 *
 */
struct AvgStdEstimator
{
  /** Reset estimator to empty series.
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

  /* Add a new value of the time series.
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

  /** Number of samples.
   *
   */
  unsigned n()
  {
    return n_;
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
   */
  std::string str()
  {
    std::ostringstream ss;
    ss << avg() << " +/- " << std() << " (max: " << max_ << ", min: " << min_ << " over " << n_ << " items)";
    return ss.str();
  }

private:
  double max_ = 0.;
  double min_ = 0.;
  double squareTotal_ = 0.;
  double total_ = 0.;
  unsigned n_ = 0;
};
