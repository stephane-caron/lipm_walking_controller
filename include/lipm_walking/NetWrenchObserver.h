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

#include <lipm_walking/Pendulum.h>
#include <lipm_walking/Contact.h>
#include <lipm_walking/defs.h>

namespace lipm_walking
{
  /** Observe net contact wrench from force/torque measurements.
   *
   */
  struct NetWrenchObserver
  {
    /** Empty constructor.
     *
     */
    NetWrenchObserver();

    /** Constructor from list of sensor names.
     *
     * \param sensorNames Identifiers of end-effector force-torque sensors.
     *
     */
    NetWrenchObserver(const std::vector<std::string> & sensorNames);

    /** Update estimates based on the sensed net contact wrench.
     *
     * \param contact Support contact frame.
     *
     */
    void update(const mc_rbdyn::Robot & robot, const Contact & contact);

    /** Net contact wrench.
     *
     */
    const sva::ForceVecd & wrench()
    {
      return netWrench_;
    }

    /** Zero-tilting moment point in the latest contact frame.
     *
     */
    const Eigen::Vector3d & zmp()
    {
      return netZMP_;
    }

  private:
    /** Update net wrench estimate from robot sensors.
     *
     * \param robot Robot state.
     *
     */
    void updateNetWrench(const mc_rbdyn::Robot & robot);

    /** Update ZMP of the net wrench.
     *
     * \param contact Frame that defines the ZMP plane.
     *
     */
    void updateNetZMP(const Contact & contact);

  private:
    Eigen::Vector3d netZMP_; /**< Net wrench ZMP in the contact frame */
    std::vector<std::string> sensorNames_ = {"LeftFootForceSensor", "RightFootForceSensor"}; /**< List of force/torque sensors */
    sva::ForceVecd netWrench_; /**< Net contact wrench */
  };
}
