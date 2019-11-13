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

#include <lipm_walking/FootstepPlan.h>

namespace lipm_walking
{
  namespace
  {
    /** Align a frame transform with the inertial horizontal plane.
     *
     * \param pose Frame to align with the horizon.
     *
     */
    sva::PTransformd makeHorizontal(sva::PTransformd pose)
    {
      const Eigen::Matrix3d R = pose.rotation();
      const Eigen::Vector3d p = pose.translation();
      Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(R);
      return {mc_rbdyn::rpyToMat(0., 0., rpy(2)), {p.x(), p.y(), 0.}};
    }
  }

  void FootstepPlan::load(const mc_rtc::Configuration & config)
  {
    config("com_height", comHeight_);
    config("contacts", contacts_);
    config("double_support_duration", doubleSupportDuration_);
    config("final_dsp_duration", finalDSPDuration_);
    config("init_dsp_duration", initDSPDuration_);
    config("landing_duration", landingDuration_);
    config("landing_pitch", landingPitch_);
    config("single_support_duration", singleSupportDuration_);
    config("swing_height", swingHeight_);
    config("takeoff_duration", takeoffDuration_);
    config("takeoff_pitch", takeoffPitch_);
    config("torso_pitch", torsoPitch_);
    if (config.has("mpc"))
    {
      mpcConfig = config("mpc");
    }
  }

  void FootstepPlan::save(mc_rtc::Configuration & config) const
  {
    config.add("com_height", comHeight_);
    config.add("contacts", contacts_);
    config.add("double_support_duration", doubleSupportDuration_);
    config.add("final_dsp_duration", finalDSPDuration_);
    config.add("init_dsp_duration", initDSPDuration_);
    config.add("landing_duration", landingDuration_);
    config.add("landing_pitch", landingPitch_);
    config.add("single_support_duration", singleSupportDuration_);
    config.add("swing_height", swingHeight_);
    config.add("takeoff_duration", takeoffDuration_);
    config.add("takeoff_pitch", takeoffPitch_);
    if (hasTorsoPitch())
    {
      config.add("torso_pitch", torsoPitch_);
    }
    if (!mpcConfig.empty())
    {
      config("mpc") = mpcConfig;
    }
  }

  void FootstepPlan::complete(const Sole & sole)
  {
    for (unsigned i = 0; i < contacts_.size(); i++)
    {
      auto & contact = contacts_[i];
      contact.id = i;
      if (contact.halfLength < 1e-4)
      {
        contact.halfLength = sole.halfLength;
      }
      if (contact.halfWidth < 1e-4)
      {
        contact.halfWidth = sole.halfWidth;
      }
      if (contact.surfaceName.length() < 1)
      {
        LOG_ERROR("Footstep plan has no surface name for contact " << i);
      }
    }
  }

  void FootstepPlan::reset(unsigned startIndex)
  {
    nextFootstep_ = startIndex + 1;
    supportContact_ = contacts_[startIndex > 0 ? startIndex - 1 : 0];
    targetContact_ = contacts_[startIndex];
    goToNextFootstep();
  }

  void FootstepPlan::goToNextFootstep()
  {
    prevContact_ = supportContact_;
    supportContact_ = targetContact_;
    unsigned targetFootstep = nextFootstep_++;
    targetContact_ = (targetFootstep < contacts_.size()) ? contacts_[targetFootstep] : prevContact_;
    nextContact_ = (nextFootstep_ < contacts_.size()) ? contacts_[nextFootstep_] : supportContact_;
  }

  void FootstepPlan::goToNextFootstep(const sva::PTransformd & actualTargetPose)
  {
    assert(nextFootstep_ >= 1);
    sva::PTransformd poseDrift = actualTargetPose * targetContact_.pose.inv();
    const Eigen::Vector3d & posDrift = poseDrift.translation();
    sva::PTransformd xyDrift = Eigen::Vector3d{posDrift.x(), posDrift.y(), 0.};
    for (unsigned i = nextFootstep_ - 1; i < contacts_.size(); i++)
    {
      contacts_[i] = xyDrift * contacts_[i];
    }
    targetContact_.pose = xyDrift * targetContact_.pose;
    goToNextFootstep();
  }

  void FootstepPlan::restorePreviousFootstep()
  {
    nextContact_ = targetContact_;
    targetContact_ = supportContact_;
    supportContact_ = prevContact_;
    nextFootstep_--;
    if (nextFootstep_ >= contacts_.size())
    {
      // at goToNextFootstep(), targetContact_ will copy prevContact_
      prevContact_ = nextContact_;
    }
  }

  sva::PTransformd FootstepPlan::computeInitialTransform(const mc_rbdyn::Robot & robot) const
  {
    sva::PTransformd X_0_c = contacts_[0].pose;
    const std::string & surfaceName = contacts_[0].surfaceName;
    const sva::PTransformd & X_0_fb = robot.posW();
    sva::PTransformd X_s_0 = robot.surfacePose(surfaceName).inv();
    sva::PTransformd X_s_fb = X_0_fb * X_s_0;
    return X_s_fb * X_0_c;
  }

  void FootstepPlan::updateInitialTransform(const sva::PTransformd & X_0_lf, const sva::PTransformd & X_0_rf, double initHeight)
  {
    sva::PTransformd X_0_mid = sva::interpolate(X_0_lf, X_0_rf, 0.5);
    sva::PTransformd X_0_old = sva::interpolate(contacts_[0].pose, contacts_[1].pose, 0.5);
    sva::PTransformd X_delta = makeHorizontal(X_0_old.inv() * X_0_mid);
    for (unsigned i = 2; i < contacts_.size(); i++)
    {
      // X_0_nc = X_old_c X_0_new = X_0_c X_old_0 X_0_new
      const sva::PTransformd & X_0_c = contacts_[i].pose;
      contacts_[i].pose = X_0_c * X_delta;
    }
    if (contacts_[0].surfaceName == "LeftFootCenter" && contacts_[1].surfaceName == "RightFootCenter")
    {
      contacts_[0].pose = makeHorizontal(X_0_lf);
      contacts_[1].pose = makeHorizontal(X_0_rf);
    }
    else if (contacts_[0].surfaceName == "RightFootCenter" && contacts_[1].surfaceName == "LeftFootCenter")
    {
      contacts_[0].pose = makeHorizontal(X_0_rf);
      contacts_[1].pose = makeHorizontal(X_0_lf);
    }
    else
    {
      LOG_ERROR("Invalid footstep plan: initial surfaces are \"" << contacts_[0].surfaceName << "\" and \"" << contacts_[1].surfaceName << "\"");
    }
    sva::PTransformd X_0_rise = Eigen::Vector3d{0., 0., initHeight};
    for (unsigned i = 0; i < contacts_.size(); i++)
    {
      contacts_[i].pose = contacts_[i].pose * X_0_rise;
    }
    X_0_init_ = X_delta * X_0_rise;
  }
}
