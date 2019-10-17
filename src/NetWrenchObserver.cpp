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

#include <lipm_walking/NetWrenchObserver.h>

namespace lipm_walking
{
  NetWrenchObserver::NetWrenchObserver()
    : sensorNames_({"LeftFootForceSensor", "RightFootForceSensor"})
  {
  }

  NetWrenchObserver::NetWrenchObserver(const std::vector<std::string> & sensorNames)
    : sensorNames_(sensorNames)
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
