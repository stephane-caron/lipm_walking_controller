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

#include <mc_rbdyn/rpy_utils.h>

#include <lipm_walking/SwingFoot.h>
#include <lipm_walking/utils/slerp.h>

namespace lipm_walking
{

void SwingFoot::reset(const sva::PTransformd & initPose,
                      const sva::PTransformd & targetPose,
                      double duration,
                      double height)
{
  assert(0. <= takeoffDuration_ && takeoffDuration_ <= 0.5 * duration);
  assert(0. <= landingDuration_ && landingDuration_ <= 0.5 * duration);

  accel_ = Eigen::Vector3d::Zero();
  aerialStart_ = takeoffDuration_;
  duration_ = duration;
  height_ = height;
  initPose_ = initPose;
  ori_ = initPose.rotation();
  playback_ = 0.;
  pos_ = initPose.translation();
  targetPose_ = targetPose;
  vel_ = Eigen::Vector3d::Zero();

  const Eigen::Vector3d & initPos = initPose_.translation();
  const Eigen::Vector3d & targetPos = targetPose_.translation();
  double aerialDuration = duration - takeoffDuration_ - landingDuration_;
  double halfDuration = duration / 2;

  Eigen::Vector3d aerialStartPos = initPos + takeoffOffset_;
  xyTakeoffChunk_.reset(initPos.head<2>(), aerialStartPos.head<2>(), takeoffDuration_);
  xyAerialChunk_.reset(aerialStartPos.head<2>(), targetPos.head<2>(), aerialDuration);

  Eigen::Vector3d airPos = 0.5 * (initPose.translation() + targetPose.translation());
  airPos.z() = std::min(initPose.translation().z(), targetPose.translation().z()) + height;
  zFirstChunk_.reset(initPos.z(), airPos.z(), halfDuration);
  zSecondChunk_.reset(airPos.z(), targetPos.z(), halfDuration);

  pitchTakeoffChunk_.reset(0., takeoffPitch_, takeoffDuration_);
  pitchAerialChunk1_.reset(takeoffPitch_, 0., aerialDuration / 2);
  pitchAerialChunk2_.reset(0., landingPitch_, aerialDuration / 2);
  pitchLandingChunk_.reset(landingPitch_, 0., landingDuration_);

  updatePose(/* t = */ 0.);
}

void SwingFoot::addLogEntries(mc_rtc::Logger & logger)
{
  logger.addLogEntry("swing_foot_accel", [this]() { return accel(); });
  logger.addLogEntry("swing_foot_offset", [this]() { return takeoffOffset_; });
  logger.addLogEntry("swing_foot_pitch", [this]() { return pitch_; });
  logger.addLogEntry("swing_foot_pose", [this]() { return pose(); });
  logger.addLogEntry("swing_foot_vel", [this]() { return vel(); });
}

void SwingFoot::removeLogEntries(mc_rtc::Logger & logger)
{
  logger.removeLogEntry("swing_foot_accel");
  logger.removeLogEntry("swing_foot_offset");
  logger.removeLogEntry("swing_foot_pitch");
  logger.removeLogEntry("swing_foot_pose");
  logger.removeLogEntry("swing_foot_vel");
}

void SwingFoot::integrate(double dt)
{
  playback_ += dt;
  updatePose(playback_);
}

void SwingFoot::updatePose(double t)
{
  updateZ(t);
  updateXY(t);
  updatePitch(t);
}

void SwingFoot::updateZ(double t)
{
  double t1 = t;
  double t2 = t - zFirstChunk_.duration();
  if(t1 <= zFirstChunk_.duration())
  {
    pos_.z() = zFirstChunk_.pos(t1);
    vel_.z() = zFirstChunk_.vel(t1);
    accel_.z() = zFirstChunk_.accel(t1);
  }
  else // (t2 > 0.)
  {
    pos_.z() = zSecondChunk_.pos(t2);
    vel_.z() = zSecondChunk_.vel(t2);
    accel_.z() = zSecondChunk_.accel(t2);
  }
}

void SwingFoot::updateXY(double t)
{
  double t1 = t - 0.;
  double t2 = t1 - xyTakeoffChunk_.duration();
  if(t1 <= xyTakeoffChunk_.duration())
  {
    pos_.head<2>() = xyTakeoffChunk_.pos(t1);
    vel_.head<2>() = xyTakeoffChunk_.vel(t1);
    accel_.head<2>() = xyTakeoffChunk_.accel(t1);
  }
  else // (t2 > 0)
  {
    pos_.head<2>() = xyAerialChunk_.pos(t2);
    vel_.head<2>() = xyAerialChunk_.vel(t2);
    accel_.head<2>() = xyAerialChunk_.accel(t2);
  }
}

void SwingFoot::updatePitch(double t)
{
  Eigen::Matrix3d baseOri;
  double t1 = t - 0.;
  double t2 = t1 - pitchTakeoffChunk_.duration();
  double t3 = t2 - pitchAerialChunk1_.duration();
  double t4 = t3 - pitchAerialChunk2_.duration();
  double takeoffDuration = pitchTakeoffChunk_.duration();
  double aerialDuration = xyAerialChunk_.duration();
  if(t1 <= takeoffDuration)
  {
    baseOri = initPose_.rotation();
  }
  else if(t2 <= aerialDuration)
  {
    double s2 = xyAerialChunk_.s(t2);
    baseOri = slerp(initPose_.rotation(), targetPose_.rotation(), s2);
  }
  else // (t2 > aerialDuration)
  {
    baseOri = targetPose_.rotation();
  }
  if(t1 <= pitchTakeoffChunk_.duration())
  {
    pitch_ = pitchTakeoffChunk_.pos(t1);
  }
  else if(t2 <= pitchAerialChunk1_.duration())
  {
    pitch_ = pitchAerialChunk1_.pos(t2);
  }
  else if(t3 <= pitchAerialChunk2_.duration())
  {
    pitch_ = pitchAerialChunk2_.pos(t3);
  }
  else // (t4 > 0)
  {
    pitch_ = pitchLandingChunk_.pos(t4);
  }
  ori_ = mc_rbdyn::rpyToMat(0., pitch_, 0.) * baseOri;
}

} // namespace lipm_walking
