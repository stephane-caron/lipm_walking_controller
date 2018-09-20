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

#include <Eigen/Dense>

namespace lipm_walking
{
  namespace world
  {
    constexpr double GRAVITY = 9.80665; // ISO 80000-3

    const Eigen::Vector3d e_z = Eigen::Vector3d{0., 0., 1.};
    const Eigen::Vector3d gravity = Eigen::Vector3d{0., 0., -GRAVITY};
  }

  const Eigen::Vector3d e_z = {0., 0., 1.};
}
