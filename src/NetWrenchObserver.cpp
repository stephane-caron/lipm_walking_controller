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

#include <lipm_walking/NetWrenchObserver.h>

namespace lipm_walking
{
  NetWrenchObserver::NetWrenchObserver(double dt)
    : dt_(dt),
      sensorNames_({"LeftFootForceSensor", "RightFootForceSensor"})
  {
  }

  NetWrenchObserver::NetWrenchObserver(double dt, const std::vector<std::string> & sensorNames)
    : dt_(dt),
      sensorNames_(sensorNames)
  {
  }

  void NetWrenchObserver::update(const mc_rbdyn::Robot & robot, const Contact & contact)
  {
    updateNetWrench(robot);
    updateNetZMP(contact);
  }

  void NetWrenchObserver::updateNetWrench(const mc_rbdyn::Robot & robot)
  {
    netWrench_ = sva::ForceVecd::Zero();
    for (std::string sensorName : sensorNames_)
    {
      const auto & sensor = robot.forceSensor(sensorName);
      if (sensor.force().z() > 1.) // pressure is more than 1 [N]
      {
        netWrench_ += sensor.worldWrench(robot);
      }
    }
  }

  void NetWrenchObserver::updateNetZMP(const Contact & contact)
  {
    const Eigen::Vector3d & force = netWrench_.force();
    const Eigen::Vector3d & moment_0 = netWrench_.couple();
    Eigen::Vector3d moment_p = moment_0 - contact.p().cross(force);
    if (force.dot(force) > 42.) // force norm is more than 5 [N]
    {
      netZMP_ = contact.p() + contact.n().cross(moment_p) / contact.n().dot(force);
    }
  }
}
