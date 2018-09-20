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

#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rbdyn/Robot.h>

namespace lipm_walking
{
  /** Kinematics-only floating-base observer.
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
    void update(mc_rbdyn::Robot & robot);

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

    /** Get anchor frame of a robot for a given contact state.
     *
     * \param robot Robot state to read frames from.
     *
     */
    sva::PTransformd getAnchorFrame(const mc_rbdyn::Robot & robot);

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
    Eigen::Matrix3d orientation_; /**< rotation from world to floating-base frame */
    Eigen::Vector3d position_; /**< translation of floating-base in world frame */
    const mc_rbdyn::Robot & controlRobot_; /**< Control robot state */
    double leftFootRatio_; /**< Fraction of total weight sustained by the left foot */
  };
}
