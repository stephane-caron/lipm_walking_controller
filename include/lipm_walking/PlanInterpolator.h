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

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/GUIState.h>

#include <lipm_walking/FootstepPlan.h>
#include <lipm_walking/utils/SE2d.h>
#include <lipm_walking/utils/polynomials.h>

namespace lipm_walking
{
  /** Footstep plan interpolator.
   *
   */
  struct PlanInterpolator
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr double DEFAULT_EXTRA_STEP_WIDTH = 0.; // [m]
    static constexpr double IN_PLACE_EXTRA_STEP_WIDTH = 0.02; // [m]
    static constexpr double IN_PLACE_MAX_STEP_ANGLE = 45.; // [deg]
    static constexpr double MAX_EXTRA_STEP_WIDTH = 0.05; // [m]
    static constexpr double MAX_LATERAL_STEP_LENGTH = 0.2; // [m]
    static constexpr double MAX_SAGITTAL_STEP_LENGTH = 0.4; // [m]
    static constexpr double MIN_STEP_LENGTH = 0.001; // [m]

    enum class Gait
    {
      Sagittal,
      Lateral,
      InPlace
    };

    /** Create footstep planner.
     *
     * \param gui State builder for the gui.
     *
     */
    PlanInterpolator(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);

    /** Add GUI panel.
     *
     */
    void addGUIElements();

    /** Read configuration from dictionary.
     *
     */
    void configure(const mc_rtc::Configuration &);

    /** Generate a new footstep plan.
     *
     */
    void run();

    /** Suggest gait based on local target coordinates.
     *
     */
    void suggestGait();

    /** Update support path display.
     *
     * \param X_0_lf Left foot transform.
     *
     * \param X_0_rf Right foot transform.
     *
     */
    void updateSupportPath(const sva::PTransformd & X_0_lf, const sva::PTransformd & X_0_rf);

    /** List available contact plans.
     *
     */
    inline std::vector<std::string> availablePlans() const
    {
      return plans_.keys();
    }

    /** Name of custom footstep plan.
     *
     */
    inline const std::string & customPlanName() const
    {
      return customPlan_.name;
    }

    /** Get current gait as string.
     *
     */
    inline std::string gait() const
    {
      if (gait_ == Gait::Sagittal)
      {
        return "Sagittal";
      }
      else if (gait_ == Gait::Lateral)
      {
        return "Lateral";
      }
      else // (gait_ == Gait::InPlace)
      {
        return "InPlace";
      }
    }

    /** Update gait.
     *
     * \param dir New gait direction.
     *
     */
    inline void gait(const std::string & dir)
    {
      if (dir == "Sagittal")
      {
        extraStepWidth_ = DEFAULT_EXTRA_STEP_WIDTH;
        gait_ = Gait::Sagittal;
      }
      else if (dir == "Lateral")
      {
        extraStepWidth_ = DEFAULT_EXTRA_STEP_WIDTH;
        gait_ = Gait::Lateral;
      }
      else // (dir == "InPlace")
      {
        extraStepWidth_ = IN_PLACE_EXTRA_STEP_WIDTH;
        gait_ = Gait::InPlace;
        targetPose_.x = 0.;
        targetPose_.y = 0.;
      }
      run();
    }

    /** Get contact plan.
     *
     */
    inline FootstepPlan getPlan(std::string name)
    {
      if (name == customPlan_.name)
      {
        return customPlan_;
      }
      else // pre-defined plans
      {
        return plans_(name);
      }
    }

    /** Remove GUI panel.
     */
    inline void removeGUIElements()
    {
      gui_->removeCategory({"Walking", "Planning"});
      gui_->removeCategory({"Walking", "Advanced", "Markers", "Planning"});
    }

    /** Restore last backward target.
     *
     */
    inline void restoreBackwardTarget()
    {
      gait_ = Gait::Sagittal;
      targetPose_ = lastBackwardTarget_;
      run();
    }

    /** Restore last forward target.
     *
     */
    inline void restoreForwardTarget()
    {
      gait_ = Gait::Sagittal;
      targetPose_ = lastForwardTarget_;
      run();
    }

    /** Restore last lateral target.
     *
     */
    inline void restoreLateralTarget()
    {
      gait_ = Gait::Lateral;
      targetPose_ = lastLateralTarget_;
      run();
    }

    /** Get step width.
     *
     */
    inline double stepWidth() const
    {
      return stepWidth_;
    }

    /** Set step width.      *
     *
     * \param stepWidth Step width.
     *
     * \note Step width is defined as the lateral distance between left and
     * right foot centers. See Figure 3-9 of Human Walking handbook (Editors J.
     * Rose and J. G. Gamble).
     *
     */
    inline void stepWidth(double stepWidth)
    {
      stepWidth_ = stepWidth;
    }

    /** Get walking target in world frame.
     *
     */
    inline const sva::PTransformd & worldReference() const
    {
      return worldReference_;
    }

    /** Set walking target in world frame.
     *
     * \param worldReference New target.
     *
     */
    inline void worldReference(const sva::PTransformd & worldReference)
    {
      worldReference_ = worldReference;
    }

  private:
    /** Restore default planning settings.
     *
     */
    void restoreDefaults();

    /** Generate a new stepping-in-place plan.
     *
     * \note Assumes it is called by run().
     *
     */
    void runInPlace_();

    /** Generate a new lateral footstep plan.
     *
     * \note Assumes it is called by run().
     *
     */
    void runLateral_();

    /** Generate a new forward footstep plan.
     *
     * \note Assumes it is called by run().
     *
     */
    void runSagittal_();

    /** Update local SE2 target from an SE2 target expressed in local frame
     *
     * \param target desired (X, Y, Theta) element in local frame
     *
     */
    void updateLocalTarget_(const SE2d& target);

    /** Update local SE2 target from world SE2 target.
     *
     * \param desired (X, Y, Theta) SE2 element expressed in world frame.
     *
     */
    void updateWorldTarget_(const Eigen::Vector3d& desired);

  public:
    unsigned nbIter = 0;

  private:
    FootstepPlan customPlan_;
    Gait gait_ = Gait::Sagittal;
    HoubaPolynomial<Eigen::Vector2d> supportPath_;
    SE2d initPose_ = {0., 0., 0.};
    SE2d lastBackwardTarget_ = {-0.5, 0., 0.};
    SE2d lastForwardTarget_ = {0.5, 0., 0.};
    SE2d lastLateralTarget_ = {0.0, 0.3, 0.};
    SE2d targetPose_ = {0.5, 0., 0.};
    bool startWithRightFootstep_ = true;
    double desiredStepAngle_ = 10. * M_PI / 180.; // [rad]
    double desiredStepLength_ = 0.2; // [m]
    double extraStepWidth_ = 0.; // [m]
    double outputVel_ = 0.25; // [m] / [s]
    double stepAngle_ = 0.0; // [deg]
    double stepLength_ = 0.2; /**< Step length in [m] */
    double stepWidth_ = 0.18; // [m], default value is for HRP-4
    mc_rtc::Configuration plans_; /**< Pre-defined footstep plans from configuration file */
    std::shared_ptr<mc_rtc::gui::StateBuilder> gui_;
    std::vector<Eigen::Vector3d> supportPathDisplay_;
    sva::PTransformd worldReference_;
    unsigned nbFootsteps_ = 0;

  private: /* Optional map */
    bool targetMap_ = false;
    std::string targetName_;
  };
}
