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
    Eigen::Matrix3d R_0_mIMU = realRobot.bodySensor().orientation().toRotationMatrix();
    Eigen::Matrix3d R_0_cBase = controlRobot_.posW().rotation();
    Eigen::Matrix3d R_0_mBase = X_rIMU_rBase.rotation() * R_0_mIMU;
    Eigen::Vector3d cRPY = mc_rbdyn::rpyFromMat(R_0_cBase);
    Eigen::Vector3d mRPY = mc_rbdyn::rpyFromMat(R_0_mBase);
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
