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

#pragma once

#include <lipm_walking/Pendulum.h>
#include <lipm_walking/defs.h>

namespace lipm_walking
{
  /** Solution to a model predictive control problem.
   *
   */
  struct Preview
  {
    /** Integrate preview on a given inverted pendulum state.
     *
     * \param pendulum Inverted pendulum model.
     *
     * \param dt Duration.
     *
     */
    virtual void integrate(Pendulum & state, double dt) = 0;

    /** Get current playback step.
     *
     */
    unsigned playbackStep() const
    {
      return playbackStep_;
    }

    /** Get current playback time.
     *
     */
    double playbackTime() const
    {
      return playbackTime_;
    }

  protected:
    double playbackTime_ = 0.;
    unsigned playbackStep_ = 0;
  };
}
