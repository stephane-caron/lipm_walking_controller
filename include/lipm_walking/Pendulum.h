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

#include <mc_tasks/CoMTask.h>

#include <lipm_walking/Contact.h>

namespace lipm_walking
{

/** State of the inverted pendulum model.
 *
 */
struct Pendulum
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Initialize state from CoM position and its derivatives.
   *
   * \param com CoM position.
   *
   * \param comd CoM velocity.
   *
   * \param comdd CoM acceleration.
   *
   */
  Pendulum(const Eigen::Vector3d & com = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

  /** Complete inverted pendulum inputs (ZMP and natural frequency) from contact plane.
   *
   * \param plane Contact plane in which the ZMP is considered.
   *
   * The current (pendulum) CoM position and acceleration are used to compute
   * the ZMP in the desired plane. When walking on horizontal ground or
   * climbing stairs, we call this function after \see resetCoMHeight to make
   * sure that the pendulum CoM is at the desired height above plane, and thus
   * that this function leaves the natural frequency \ref omega_ untouched.
   *
   */
  void completeIPM(const Contact & plane);

  /** Integrate constant CoM jerk for a given duration.
   *
   * \param comddd CoM jerk.
   *
   * \param dt Integration step.
   *
   */
  void integrateCoMJerk(const Eigen::Vector3d & comddd, double dt);

  /** Integrate in floating-base inverted pendulum mode with constant inputs.
   *
   * \param zmp Zero-tilting Moment Point, i.e. net force application point.
   *
   * \param lambda Normalized stiffness of the pendulum.
   *
   * \param dt Duration of integration step.
   *
   */
  void integrateIPM(Eigen::Vector3d zmp, double lambda, double dt);

  /** Reset to a new state from CoM position and its derivatives.
   *
   * \param com CoM position.
   *
   * \param comd CoM velocity.
   *
   * \param comdd CoM acceleration.
   *
   */
  void reset(const Eigen::Vector3d & com,
             const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
             const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

  /** Reset CoM height above a given contact plane.
   *
   * \param height CoM height above contact plane.
   *
   * \param contact Contact plane.
   *
   * We call this function before \see completeIPM when using the smooth
   * continuous trajectory generator implemented in \ref
   * ModelPredictiveControl. The combination of these two functions preserves
   * continuity of the horizontal coordinates of the ZMP trajectory, while
   * adapting its height to the contact height.
   *
   */
  void resetCoMHeight(double height, const Contact & contact);

  /** CoM position in the world frame.
   *
   */
  const Eigen::Vector3d & com() const
  {
    return com_;
  }

  /** CoM velocity in the world frame.
   *
   */
  const Eigen::Vector3d & comd() const
  {
    return comd_;
  }

  /** CoM acceleration in the world frame.
   *
   */
  const Eigen::Vector3d & comdd() const
  {
    return comdd_;
  }

  /** Divergent component of motion.
   *
   */
  Eigen::Vector3d dcm() const
  {
    return com_ + comd_ / omega_;
  }

  /** Natural frequency.
   *
   */
  double omega() const
  {
    return omega_;
  }

  /** Zero-tilting moment point.
   *
   * \note In the linear inverted pendulum mode, the ZMP coincides with the
   * centroidal moment pivot (CMP) or its extended version (eCMP).
   *
   */
  const Eigen::Vector3d & zmp() const
  {
    return zmp_;
  }

  /** Velocity of the zero-tilting moment point.
   *
   */
  const Eigen::Vector3d & zmpd() const
  {
    return zmpd_;
  }

protected:
  Eigen::Vector3d com_; /**< Position of the center of mass */
  Eigen::Vector3d comd_; /**< Velocity of the center of mass */
  Eigen::Vector3d comdd_; /**< Acceleration of the center of mass */
  Eigen::Vector3d comddd_; /**< Jerk of the center of mass */
  Eigen::Vector3d zmp_; /**< Position of the zero-tilting moment point */
  Eigen::Vector3d zmpd_; /**< Velocity of the zero-tilting moment point */
  double omega_; /**< Natural frequency in [Hz] */
};

} // namespace lipm_walking
