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
  /** MPC parameters.
   *
   * These parameters are shared between MPC problems and solutions.
   *
   */
  namespace HorizontalMPC
  {
    constexpr double SAMPLING_PERIOD = 0.1; // [s]
    constexpr unsigned INPUT_SIZE = 2; // input is 2D CoM jerk
    constexpr unsigned NB_STEPS = 16; // number of sampling steps
    constexpr unsigned STATE_SIZE = 6; // state is CoM [pos, vel, accel]
  }
}
