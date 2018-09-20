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

/** Spherical linear interpolation between two rotation matrices.
 *
 * \param from First rotation matrix.
 *
 * \param to Second rotation matrix.
 *
 * \param t Interpolation index.
 *
 * \returns R Interpolation between those two matrices.
 *
 * See <https://en.wikipedia.org/wiki/Slerp>.
 *
 */
inline Eigen::Matrix3d slerp(const Eigen::Matrix3d & from, const Eigen::Matrix3d & to, double t)
{
  Eigen::Quaterniond qFrom(from);
  Eigen::Quaterniond qTo(to);
  return qFrom.slerp(t, qTo).toRotationMatrix();
}
