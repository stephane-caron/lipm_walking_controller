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

#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rtc/log/Logger.h>

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
}
