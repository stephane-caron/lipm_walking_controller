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
#include <mc_rbdyn/rpy_utils.h>

namespace utils
{
  /** SE2 transform.
   *
   */
  struct SE2d
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Initialize a new SE2 transform.
     *
     * \param x First translation coordinate.
     *
     * \param y Second translation coordinate.
     *
     * \param theta Orientation coordinate.
     *
     */
    SE2d(double x = 0., double y = 0., double theta = 0.)
      : x(x), y(y), theta(theta) {}
  
    /** Apply SE2 transform in horizontal plane of an SE3 frame.
     *
     * \param X_0_a Plucker transform of SE3 frame.
     *
     */
    inline sva::PTransformd operator*(const sva::PTransformd & X_0_a) const
    {
      sva::PTransformd X_a_b = asPTransform();
      return X_a_b * X_0_a;
    }
  
    /** Convert to Plucker transform.
     *
     */
    inline sva::PTransformd asPTransform() const
    {
      return {mc_rbdyn::rpyToMat(0., 0., theta), Eigen::Vector3d{x, y, 0.}};
    }
  
    /** Get 2D position.
     *
     * \returns pos Position vector {x, y}.
     *
     */
    inline Eigen::Vector2d pos() const
    {
      return {x, y};
    }
  
    /** Get 2D orientation vector.
     *
     * \returns t Orientation vector {cos(theta), sin(theta)}.
     *
     */
    inline Eigen::Vector2d ori() const
    {
      return {std::cos(theta), std::sin(theta)};
    }
  
    /** Get transform in vector form.
     *
     * \returns v Vector {x, y, theta} of transform coordinates.
     *
     */
    inline Eigen::Vector3d vector()
    {
      return {x, y, theta};
    }
  
    /** Get transform in vector form, with the angle expressed in degrees.
     *
     * \returns v Vector {x, y, theta[deg]} of transform coordinates.
     *
     */
    inline Eigen::Vector3d vectorDegrees() const
    {
      return {x, y, theta * 180. / M_PI};
    }
  
  public:
    double x; /**< First translation coordinate. */
    double y; /**< Second translation coordinate. */
    double theta; /**< Orientation coordinate. */
  };
}

using utils::SE2d;
