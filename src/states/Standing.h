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

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/State.h>

namespace lipm_walking
{

namespace states
{

/** Enable stabilizer and keep CoM at a reference position.
 *
 * Applies a simple CoM set-point task:
 *
 * \f[
 *   \ddot{c} = K (c^d - c) - B \dot{c}
 * \f]
 *
 * with critical damping \f$B = 2 \sqrt{K}\f$.
 *
 */
struct Standing : State
{
  /** Start state.
   *
   */
  void start() override;

  /** Teardown state.
   *
   */
  void teardown() override;

  /** Check for footstep plan updates.
   *
   */
  void checkPlanUpdates();

  /** Check transitions at beginning of control cycle.
   *
   */
  bool checkTransitions() override;

  /** Main state function, called if no transition at this cycle.
   *
   */
  void runState() override;

  /** Distribute spatial ZMP into foot CoPs in double support.
   *
   */
  void distributeFootCoPs();

  /** Update target CoM and CoP.
   *
   * \param leftFootRatio Left foot weight index between 0 and 1.
   *
   */
  void updateTarget(double leftFootRatio);

  /** Make foot contact.
   *
   * \param footTask Stabilizer task corresponding to the foot to release.
   *
   * \param contact Contact target.
   *
   */
  void makeFootContact(std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact);

  /** Make left foot contact.
   *
   */
  void makeLeftFootContact();

  /** Make right foot contact.
   *
   */
  void makeRightFootContact();

  /** Release foot contact.
   *
   * \param footTask Stabilizer task corresponding to the foot to release.
   *
   * \return True if the contact was released, false if not.
   */
  bool releaseFootContact(std::shared_ptr<mc_tasks::force::CoPTask> footTask);

  /** Release left foot contact.
   *
   */
  void releaseLeftFootContact();

  /** Release right foot contact.
   *
   */
  void releaseRightFootContact();

  /** Enable startWalking_ boolean and update GUI.
   *
   */
  void startWalking();

protected:
  /** Change footstep plan.
   *
   * \param name New plan name.
   *
   */
  void updatePlan(const std::string & name);

private:
  Contact leftFootContact_; /**< Current left foot contact handle in plan */
  Contact rightFootContact_; /**< Current right foot contact handle in plan */
  Eigen::Vector3d copTarget_; /**< CoP target computed from GUI input */
  bool isMakingFootContact_; /**< Is the robot going back to double support? */
  bool planChanged_; /**< Has footstep plan changed? */
  bool startWalking_; /**< Has the user clicked on "Start walking"? */
  double freeFootGain_; /**< Foot task gain when lifting one foot in the air */
  double leftFootRatio_; /**< Left foot ratio from GUI input */
  double releaseHeight_; /**< Desired height when lifting one foot in the air */
  unsigned lastInterpolatorIter_; /**< Last iteration number of the plan interpolator */
};

} // namespace states

} // namespace lipm_walking
