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
    /** Constructor.
     *
     * \param dt Controller time step.
     *
     */
    NetWrenchObserver(double dt)
      : dt_(dt),
        sensorNames_({"LeftFootForceSensor", "RightFootForceSensor"})
    {
    }

    /** Constructor.
     *
     * \param dt Controller time step.
     *
     * \param sensorNames Identifiers of end-effector force-torque sensors.
     *
     */
    NetWrenchObserver(double dt, const std::vector<std::string> & sensorNames)
      : dt_(dt),
        sensorNames_(sensorNames)
    {
    }

    /** Net contact wrench.
     *
     */
    inline const sva::ForceVecd & wrench()
    {
      return netWrench_;
    }

    /** Zero-tilting moment point in the latest contact frame.
     *
     */
    inline const Eigen::Vector3d & zmp()
    {
      return netZMP_;
    }

    /** Update estimates based on the sensed net contact wrench.
     *
     * \param contact Support contact frame.
     *
     */
    void update(const mc_rbdyn::Robot & robot, const Contact & contact);

  private:
    void updateNetWrench(const mc_rbdyn::Robot & robot);

    void updateNetZMP(const Contact & contact);

  private:
    Eigen::Vector3d netZMP_;
    double dt_;
    std::vector<std::string> sensorNames_;
    sva::ForceVecd netWrench_;
  };
}
