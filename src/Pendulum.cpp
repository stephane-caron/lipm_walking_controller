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

#include <lipm_walking/Pendulum.h>
#include <lipm_walking/utils/world.h>

namespace lipm_walking
{
  Pendulum::Pendulum(const Eigen::Vector3d & com, const Eigen::Vector3d & comd, const Eigen::Vector3d & comdd)
  {
    reset(com, comd, comdd);
  }

  void Pendulum::reset(const Eigen::Vector3d & com, const Eigen::Vector3d & comd, const Eigen::Vector3d & comdd)
  {
    constexpr double DEFAULT_HEIGHT = 0.8; // [m]
    constexpr double DEFAULT_LAMBDA = world::GRAVITY / DEFAULT_HEIGHT;
    com_ = com;
    comd_ = comd;
    comdd_ = comdd;
    comddd_ = Eigen::Vector3d::Zero();
    omega_ = std::sqrt(DEFAULT_LAMBDA);
    zmp_ = com_ + (world::gravity - comdd_) / DEFAULT_LAMBDA;
    zmpd_ = comd_ - comddd_ / DEFAULT_LAMBDA;
  }

  void Pendulum::integrateIPM(Eigen::Vector3d zmp, double lambda, double dt)
  {
    Eigen::Vector3d com_prev = com_;
    Eigen::Vector3d comd_prev = comd_;
    omega_ = std::sqrt(lambda);
    zmp_ = zmp;

    Eigen::Vector3d vrp = zmp_ - world::gravity / lambda;
    double ch = std::cosh(omega_ * dt);
    double sh = std::sinh(omega_ * dt);
    comdd_ = lambda * (com_prev - zmp_) + world::gravity;
    comd_ = comd_prev * ch + omega_ * (com_prev - vrp) * sh;
    com_ = com_prev * ch + comd_prev * sh / omega_ - vrp * (ch - 1.0);

    // default values for third-order terms
    comddd_ = Eigen::Vector3d::Zero();
    zmpd_ = comd_ - comddd_ / lambda;
  }

  void Pendulum::integrateCoMJerk(const Eigen::Vector3d & comddd, double dt)
  {
    com_ += dt * (comd_ + dt * (comdd_ / 2 + dt * (comddd / 6)));
    comd_ += dt * (comdd_ + dt * (comddd / 2));
    comdd_ += dt * comddd;
    comddd_ = comddd;
  }

  void Pendulum::resetCoMHeight(double height, const Contact & plane)
  {
    auto n = plane.normal();
    com_ += (height + n.dot(plane.p() - com_)) * n;
    comd_ -= n.dot(comd_) * n;
    comdd_ -= n.dot(comdd_) * n;
    comddd_ -= n.dot(comddd_) * n;
  }

  void Pendulum::completeIPM(const Contact & plane)
  {
    auto n = plane.normal();
    auto gravitoInertial = world::gravity - comdd_;
    double lambda = n.dot(gravitoInertial) / n.dot(plane.p() - com_);
    zmp_ = com_ + gravitoInertial / lambda;
    zmpd_ = comd_ - comddd_ / lambda;
    omega_ = std::sqrt(lambda);
  }
}
