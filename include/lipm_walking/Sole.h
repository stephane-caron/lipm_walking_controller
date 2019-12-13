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

#include <mc_rtc/Configuration.h>

namespace lipm_walking
{

/** Foot sole properties.
 *
 */
struct Sole
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2d leftAnkleOffset =
      Eigen::Vector2d::Zero(); /**< Offset from center to ankle frames in the left sole frame */
  double friction = 0.7; /**< Friction coefficient */
  double halfLength = 0.112; /**< Half-length of the sole in [m] */
  double halfWidth = 0.065; /**< Half-width of the sole in [m]*/
};

} // namespace lipm_walking

/** Framework namespace, only used to add a configuration loader.
 *
 */
namespace mc_rtc
{

template<>
struct ConfigurationLoader<lipm_walking::Sole>
{
  static lipm_walking::Sole load(const mc_rtc::Configuration & config)
  {
    lipm_walking::Sole sole;
    config("friction", sole.friction);
    config("half_length", sole.halfLength);
    config("half_width", sole.halfWidth);
    return sole;
  }

  static mc_rtc::Configuration save(const lipm_walking::Sole & sole)
  {
    mc_rtc::Configuration config;
    config.add("friction", sole.friction);
    config.add("half_length", sole.halfLength);
    config.add("half_width", sole.halfWidth);
    return config;
  }
};

} // namespace mc_rtc
