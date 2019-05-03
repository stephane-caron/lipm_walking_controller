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

    /** Default CoM height.
     *
     */
    double comHeight() const
    {
      return comHeight_;
    }

    /** Set default CoM height.
     *
     */
    void comHeight(double height)
    {
      constexpr double MIN_COM_HEIGHT = 0.7; // [m]
      constexpr double MAX_COM_HEIGHT = 0.85; // [m]
      comHeight_ = clamp(height, MIN_COM_HEIGHT, MAX_COM_HEIGHT);
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
      finalDSPDuration_ = clamp(duration, 0., 1.6);
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
      initDSPDuration_ = clamp(duration, 0., 1.6);
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
    inline void rewind()
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
    double comHeight_ = 0.78; // [m]
    double doubleSupportDuration_ = 0.2; // [s]
    double finalDSPDuration_ = 0.3; // [s]
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
