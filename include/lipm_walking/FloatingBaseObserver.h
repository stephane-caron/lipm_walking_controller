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

#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rbdyn/Robot.h>

namespace lipm_walking
{
  /** Kinematics-only floating-base observer.
   *
   * See <https://scaron.info/teaching/floating-base-estimation.html> for
   * technical details on the derivation of this simple estimator.
   *
   */
  struct FloatingBaseObserver
  {
    /** Initialize floating base observer.
     *
     * \param controlRobot Robot reference.
     *
     */
    FloatingBaseObserver(const mc_rbdyn::Robot & controlRobot);

    /** Get anchor frame of a robot for a given contact state.
     *
     * \param robot Robot state to read frames from.
     *
     */
    sva::PTransformd getAnchorFrame(const mc_rbdyn::Robot & robot);

    /** Reset floating base estimate.
     *
     * \param X_0_fb New floating-base transform.
     *
     */
    void reset(const sva::PTransformd & X_0_fb);

    /** Update floating-base transform of real robot.
     *
     * \param realRobot Measured robot state, to be updated.
     *
     */
    void run(const mc_rbdyn::Robot & realRobot);

    /** Write observed floating-base transform to the robot's configuration.
     *
     * \param robot Robot state to write to.
     *
     */
    void updateRobot(mc_rbdyn::Robot & robot);

    /** Set fraction of total weight sustained by the left foot.
     *
     * \note This field is used in anchor frame computations.
     *
     */
    void leftFootRatio(double ratio)
    {
      leftFootRatio_ = ratio;
    }

    /** Get floating-base pose in the world frame.
     *
     */
    sva::PTransformd posW()
    {
      return {orientation_, position_};
    }

  private:
    /** Update floating-base orientation based on new observed gravity vector.
     *
     * \param realRobot Measured robot state.
     *
     */
    void estimateOrientation(const mc_rbdyn::Robot & realRobot);

    /* Update floating-base position.
     *
     * \param realRobot Measurements robot model.
     *
     * The new position is chosen so that the origin of the real anchor frame
     * coincides with the control anchor frame.
     *
     */
    void estimatePosition(const mc_rbdyn::Robot & realRobot);

  private:
    Eigen::Matrix3d orientation_; /**< Rotation from world to floating-base frame */
    Eigen::Vector3d position_; /**< Translation of floating-base in world frame */
    const mc_rbdyn::Robot & controlRobot_; /**< Control robot state */
    double leftFootRatio_; /**< Fraction of total weight sustained by the left foot */
  };
}
