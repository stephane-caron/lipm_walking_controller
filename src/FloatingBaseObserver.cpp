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

#include <cmath>

#include <mc_rbdyn/rpy_utils.h>

#include <lipm_walking/FloatingBaseObserver.h>

namespace lipm_walking
{
  FloatingBaseObserver::FloatingBaseObserver(const mc_rbdyn::Robot & controlRobot)
    : controlRobot_(controlRobot)
  {
  }

  void FloatingBaseObserver::reset(const sva::PTransformd & X_0_fb)
  {
    orientation_ = X_0_fb.rotation();
    position_ = X_0_fb.translation();
  }

  void FloatingBaseObserver::run(const mc_rbdyn::Robot & realRobot)
  {
    estimateOrientation(realRobot);
    estimatePosition(realRobot);
  }

  void FloatingBaseObserver::estimateOrientation(const mc_rbdyn::Robot & realRobot)
  {
    // Prefixes:
    // c for control-robot model
    // r for real-robot model
    // m for estimated/measured quantities
    sva::PTransformd X_0_rBase = realRobot.posW();
    sva::PTransformd X_0_rIMU = realRobot.bodyPosW(realRobot.bodySensor().parentBody());
    sva::PTransformd X_rIMU_rBase = X_0_rBase * X_0_rIMU.inv();
    Eigen::Matrix3d E_0_mIMU = realRobot.bodySensor().orientation().toRotationMatrix();
    Eigen::Matrix3d E_0_cBase = controlRobot_.posW().rotation();
    Eigen::Matrix3d E_0_mBase = X_rIMU_rBase.rotation() * E_0_mIMU;
    Eigen::Vector3d cRPY = mc_rbdyn::rpyFromMat(E_0_cBase);
    Eigen::Vector3d mRPY = mc_rbdyn::rpyFromMat(E_0_mBase);
    orientation_ = mc_rbdyn::rpyToMat(mRPY(0), mRPY(1), cRPY(2));
  }

  void FloatingBaseObserver::estimatePosition(const mc_rbdyn::Robot & realRobot)
  {
    sva::PTransformd X_0_c = getAnchorFrame(controlRobot_);
    sva::PTransformd X_0_s = getAnchorFrame(realRobot);
    const sva::PTransformd & X_0_real = realRobot.posW();
    sva::PTransformd X_real_s = X_0_s * X_0_real.inv();
    const Eigen::Vector3d & r_c_0 = X_0_c.translation();
    const Eigen::Vector3d & r_s_real = X_real_s.translation();
    position_ = r_c_0 - orientation_.transpose() * r_s_real;
  }

  sva::PTransformd FloatingBaseObserver::getAnchorFrame(const mc_rbdyn::Robot & robot)
  {
    sva::PTransformd X_0_l = robot.surface("LeftFoot").X_0_s(robot);
    sva::PTransformd X_0_r = robot.surface("RightFoot").X_0_s(robot);
    return sva::interpolate(X_0_r, X_0_l, leftFootRatio_);
  }

  void FloatingBaseObserver::updateRobot(mc_rbdyn::Robot & realRobot)
  {
    realRobot.posW(sva::PTransformd{orientation_, position_});
  }
}
