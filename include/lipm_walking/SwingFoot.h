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

#include <mc_rtc/log/Logger.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <lipm_walking/utils/polynomials.h>

namespace lipm_walking
{

/** Swing foot interpolator.
 *
 */
struct SwingFoot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Add swing foot entries to log.
   *
   * \param logger Logger.
   *
   */
  void addLogEntries(mc_rtc::Logger & logger);

  /** Progress by dt along the swing foot trajectory.
   *
   * \param dt Integration step.
   *
   */
  void integrate(double dt);

  /** Remove swing foot entries from log.
   *
   * \param logger Logger.
   *
   */
  void removeLogEntries(mc_rtc::Logger & logger);

  /** Recompute swing foot trajectory for a new pair of contacts.
   *
   * \param initPose Pose that the swing foot starts from.
   *
   * \param targetPose Target pose the swing foot lands onto.
   *
   * \param duration Duration of the trajectory.
   *
   * \param height Apex height of swing trajectory.
   *
   */
  void reset(const sva::PTransformd & initPose, const sva::PTransformd & targetPose, double duration, double height);

  /** Get current acceleration as motion vector.
   *
   */
  sva::MotionVecd accel() const
  {
    return sva::MotionVecd({0., 0., 0.}, accel_);
  }

  /** Get current swing foot height.
   *
   */
  double height()
  {
    return pos_.z() - initPose_.translation().z();
  }

  /** Duration of landing phase.
   *
   * \param duration New duration.
   *
   */
  void landingDuration(double duration)
  {
    landingDuration_ = duration;
  }

  /** Upward pitch angle before landing.
   *
   * \param pitch Pitch angle.
   *
   */
  void landingPitch(double pitch)
  {
    landingPitch_ = pitch;
  }

  /** Get current pose as Plucker transform.
   *
   */
  sva::PTransformd pose() const
  {
    return sva::PTransformd(ori_, pos_);
  }

  /** Timing remaining until heel strike.
   *
   */
  double remTime() const
  {
    return (duration_ - playback_);
  }

  /** Set duration of takeoff phase.
   *
   * \param duration New duration.
   *
   */
  void takeoffDuration(double duration)
  {
    takeoffDuration_ = duration;
  }

  /** Offset applied to horizontal position after takeoff.
   *
   * \param offset Offset.
   *
   */
  void takeoffOffset(const Eigen::Vector3d & offset)
  {
    takeoffOffset_ = offset;
  }

  /** Downward pitch angle after takeoff.
   *
   * \param pitch Pitch angle.
   *
   */
  void takeoffPitch(double pitch)
  {
    takeoffPitch_ = pitch;
  }

  /** Get current velocity as motion vector.
   *
   */
  sva::MotionVecd vel() const
  {
    return sva::MotionVecd({0., 0., 0.}, vel_);
  }

private:
  /** Update pose to a given playback time.
   *
   * \param t Playback time.
   *
   */
  void updatePose(double t);

  /** Update altitude to a given playback time.
   *
   * \param t Playback time.
   *
   */
  void updateZ(double t);

  /** Update xy-position to a given playback time.
   *
   * \param t Playback time.
   *
   */
  void updateXY(double t);

  /** Update pitch angle to a given playback time.
   *
   * \param t Playback time.
   *
   */
  void updatePitch(double t);

private:
  Eigen::Quaterniond ori_;
  Eigen::Vector3d accel_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d takeoffOffset_ = Eigen::Vector3d::Zero(); // [m]
  Eigen::Vector3d vel_;
  RetimedPolynomial<QuinticHermitePolynomial, Eigen::Vector2d> xyAerialChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, Eigen::Vector2d> xyTakeoffChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, double> pitchAerialChunk1_;
  RetimedPolynomial<QuinticHermitePolynomial, double> pitchAerialChunk2_;
  RetimedPolynomial<QuinticHermitePolynomial, double> pitchLandingChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, double> pitchTakeoffChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, double> zFirstChunk_;
  RetimedPolynomial<QuinticHermitePolynomial, double> zSecondChunk_;
  double aerialStart_;
  double duration_;
  double height_;
  double landingDuration_ = 0.; // [s]
  double landingPitch_ = 0.; // [rad]
  double pitch_;
  double playback_;
  double takeoffDuration_ = 0.; // [s]
  double takeoffPitch_ = 0.; // [rad]
  sva::PTransformd initPose_;
  sva::PTransformd targetPose_;
  sva::PTransformd touchdownPose_;
};

} // namespace lipm_walking
