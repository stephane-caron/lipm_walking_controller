/* Copyright 2018 CNRS-UM LIRMM
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

namespace lipm_walking
{
  /** Foot sole properties.
   *
   */
  struct Sole
  {
    double friction = 0.7;
    double halfLength = 0.112; // [m]
    double halfWidth = 0.065; // [m]
  };
}

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
}
