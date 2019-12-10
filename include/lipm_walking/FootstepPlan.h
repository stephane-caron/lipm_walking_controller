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

#include <string>

#include <lipm_walking/Contact.h>
#include <lipm_walking/Sole.h>
#include <lipm_walking/utils/clamp.h>

namespace lipm_walking
{
  /** Sequence of footsteps with gait parameters.
   *
   */
  struct FootstepPlan
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Complete contacts from sole parameters.
     *
     * \param sole Sole parameters.
     *
     */
    void complete(const Sole & sole);

    /** Compute initial floating-base transform over first contact.
     *
     * \param robot Robot model.
     *
     */
    sva::PTransformd computeInitialTransform(const mc_rbdyn::Robot & robot) const;

    /** Advance to next footstep in plan.
     *
     */
    void goToNextFootstep();

    /** Advance to next footstep in plan, taking into account drift in reaching
     * target contact.
     *
     * \param actualTargetPose Actual target pose.
     *
     */
    void goToNextFootstep(const sva::PTransformd & actualTargetPose);

    /** Load plan from configuration dictionary.
     *
     * \param config Configuration dictionary.
     *
     */
    void load(const mc_rtc::Configuration & config);

    /** Rewind plan to a given contact.
     *
     * \param startIndex Index of first support contact.
     *
     */
    void reset(unsigned startIndex);

    /** Rewind one footstep back in plan.
     *
     * \note This function cannot rewind more than one step. It is only used
     * when activating a DoubleSupport to Standing transition.
     */
    void restorePreviousFootstep();

    /** Save plan to configuration  dictionary.
     *
     * \param config Configuration dictionary.
     *
     */
    void save(mc_rtc::Configuration & config) const;

    /** Update plan coordinates from robot foot transforms.
     *
     * \param X_0_lf Left foot transform.
     *
     * \param X_0_rf Right foot transform.
     *
     * \param initHeight Height of first two contacts after transform.
     *
     */
    void updateInitialTransform(const sva::PTransformd & X_0_lf, const sva::PTransformd & X_0_rf, double initHeight);

    /** Append footstep to plan.
     *
     */
    void appendContact(Contact step)
    {
      contacts_.push_back(step);
    }

    /** Default CoM height above ground contact.
     *
     */
    double comHeight() const
    {
      return comHeight_;
    }

    /** Set default CoM height above ground contact.
     *
     */
    void comHeight(double height)
    {
      comHeight_ = clamp(height, 0., 2.);
    }

    /** Reference to list of contacts.
     *
     */
    const std::vector<Contact> & contacts() const
    {
      return contacts_;
    }

    /** Default double-support duration.
     *
     */
    double doubleSupportDuration() const
    {
      return doubleSupportDuration_;
    }

    /** Set default double-support duration.
     *
     * \param duration New duration.
     *
     */
    void doubleSupportDuration(double duration)
    {
      doubleSupportDuration_ = clamp(duration, 0., 1.);
    }

    /** Get final double support phase duration.
     *
     */
    double finalDSPDuration() const
    {
      return finalDSPDuration_;
    }

    /** Does the plan provide a reference torso pitch?
     *
     */
    bool hasTorsoPitch() const
    {
      return (torsoPitch_ > -10.);
    }

    /** Set final double support phase duration.
     *
     * \param duration New duration.
     *
     */
    void finalDSPDuration(double duration)
    {
      finalDSPDuration_ = clamp(duration, 0.1, 1.6);
    }

    /** Get initial double support phase duration.
     *
     */
    double initDSPDuration() const
    {
      return initDSPDuration_;
    }

    /** Set initial double support phase duration.
     *
     */
    void initDSPDuration(double duration)
    {
      initDSPDuration_ = clamp(duration, 0.1, 1.6);
    }

    /** Initial transform with respect to world frame.
     *
     * \returns X_0_init Initial transform of the plan.
     *
     */
    const sva::PTransformd & initPose()
    {
      return X_0_init_;
    }

    /** Get swing foot landing ratio.
     *
     */
    double landingDuration() const
    {
      if (supportContact_.swingConfig.has("landing_duration"))
      {
        return supportContact_.swingConfig("landing_duration");
      }
      return landingDuration_;
    }

    /** Set swing foot landing duration.
     *
     * \param duration New duration.
     *
     */
    void landingDuration(double duration)
    {
      landingDuration_ = clamp(duration, 0., 0.5);
    }

    /** Get swing foot landing pitch angle.
     *
     */
    double landingPitch() const
    {
      if (prevContact_.swingConfig.has("landing_pitch"))
      {
        return prevContact_.swingConfig("landing_pitch");
      }
      return landingPitch_;
    }

    /** Set swing foot takeoff pitch angle.
     *
     */
    void landingPitch(double pitch)
    {
      constexpr double MIN_LANDING_PITCH = -1.;
      constexpr double MAX_LANDING_PITCH = 1.;
      landingPitch_ = clamp(pitch, MIN_LANDING_PITCH, MAX_LANDING_PITCH);
    }

    /** Next contact in plan.
     *
     */
    const Contact & nextContact() const
    {
      return nextContact_;
    }

    /** Previous contact in plan.
     *
     */
    const Contact & prevContact() const
    {
      return prevContact_;
    }

    /** Reset contacts from another footstep plan.
     *
     * \param contacts New sequence of contacts.
     *
     */
    void resetContacts(const std::vector<Contact> & contacts)
    {
      contacts_ = contacts;
    }

    /** Default single-support duration.
     *
     */
    double singleSupportDuration() const
    {
      return singleSupportDuration_;
    }

    /** Set single-support duration.
     *
     */
    void singleSupportDuration(double duration)
    {
      singleSupportDuration_ = clamp(duration, 0., 2.);
    }

    /** Current support contact.
     *
     */
    const Contact & supportContact() const
    {
      return supportContact_;
    }

    /** Swap first two contacts in plan.
     *
     */
    void swapFirstTwoContacts()
    {
      std::swap(contacts_[0], contacts_[1]);
    }

    /** Default swing-foot height.
     *
     */
    double swingHeight() const
    {
      if (prevContact_.swingConfig.has("height"))
      {
        return prevContact_.swingConfig("height");
      }
      return swingHeight_;
    }

    /** Set default swing-foot height.
     *
     */
    void swingHeight(double height)
    {
      constexpr double MIN_SWING_FOOT_HEIGHT = 0.;
      constexpr double MAX_SWING_FOOT_HEIGHT = 0.25;
      swingHeight_ = clamp(height, MIN_SWING_FOOT_HEIGHT, MAX_SWING_FOOT_HEIGHT);
    }

    /** Get swing foot takeoff duration.
     *
     */
    double takeoffDuration() const
    {
      if (supportContact_.swingConfig.has("takeoff_duration"))
      {
        return supportContact_.swingConfig("takeoff_duration");
      }
      return takeoffDuration_;
    }

    /** Set swing foot takeoff duration.
     *
     * \param duration New duration.
     *
     */
    void takeoffDuration(double duration)
    {
      takeoffDuration_ = clamp(duration, 0., 0.5);
    }

    /** Get swing foot takeoff offset.
     *
     */
    Eigen::Vector3d takeoffOffset() const
    {
      if (prevContact_.swingConfig.has("takeoff_offset"))
      {
        return prevContact_.swingConfig("takeoff_offset");
      }
      return takeoffOffset_;
    }

    /** Set swing foot takeoff offset.
     *
     */
    void takeoffOffset(const Eigen::Vector3d & offset)
    {
      takeoffOffset_ = offset;
    }

    /** Get swing foot takeoff pitch angle.
     *
     */
    double takeoffPitch() const
    {
      if (prevContact_.swingConfig.has("takeoff_pitch"))
      {
        return prevContact_.swingConfig("takeoff_pitch");
      }
      return takeoffPitch_;
    }

    /** Set swing foot takeoff pitch angle.
     *
     */
    void takeoffPitch(double pitch)
    {
      constexpr double MIN_TAKEOFF_PITCH = -1.;
      constexpr double MAX_TAKEOFF_PITCH = 1.;
      takeoffPitch_ = clamp(pitch, MIN_TAKEOFF_PITCH, MAX_TAKEOFF_PITCH);
    }

    /** Current target contact.
     *
     */
    const Contact & targetContact() const
    {
      return targetContact_;
    }

    /** Reference torso pitch angle.
     *
     */
    double torsoPitch() const
    {
      return torsoPitch_;
    }

    /** Rewind plan to the beginning.
     *
     */
    void rewind()
    {
      reset(0);
    }

  public:
    mc_rtc::Configuration mpcConfig;
    std::string name = "";

  private:
    Contact nextContact_;
    Contact prevContact_;
    Contact supportContact_;
    Contact targetContact_;
    Eigen::Vector3d takeoffOffset_ = Eigen::Vector3d::Zero(); // [m]
    double comHeight_ = 0.8; // [m]
    double doubleSupportDuration_ = 0.2; // [s]
    double finalDSPDuration_ = 0.6; // [s]
    double initDSPDuration_ = 0.6; // [s]
    double landingDuration_ = 0.15; // [s]
    double landingPitch_ = 0.;
    double singleSupportDuration_ = 0.8; // [s]
    double swingHeight_ = 0.04; // [m]
    double takeoffDuration_ = 0.05; // [s]
    double takeoffPitch_ = 0.;
    double torsoPitch_ = -100.;
    std::vector<Contact> contacts_;
    sva::PTransformd X_0_init_;
    unsigned nextFootstep_ = 0;
  };
}

namespace mc_rtc
{
  template<>
  struct ConfigurationLoader<lipm_walking::FootstepPlan>
  {
    static lipm_walking::FootstepPlan load(const mc_rtc::Configuration & config)
    {
      lipm_walking::FootstepPlan plan;
      plan.load(config);
      return plan;
    }

    static mc_rtc::Configuration save(const lipm_walking::FootstepPlan & plan)
    {
      mc_rtc::Configuration config;
      plan.save(config);
      return config;
    }
  };
}
