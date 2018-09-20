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

#include <mc_rtc/logging.h>

/** Clamp a value in a given interval.
 *
 * \param v Value.
 *
 * \param vMin Lower bound.
 *
 * \param vMax Upper bound.
 *
 */
inline double clamp(double v, double vMin, double vMax)
{
  if (v > vMax)
  {
    return vMax;
  }
  else if (v < vMin)
  {
    return vMin;
  }
  else
  {
    return v;
  }
}


/** Clamp a value in a given interval.
 *
 * \param v Reference to value.
 *
 * \param vMin Lower bound.
 *
 * \param vMax Upper bound.
 *
 */
inline void clampInPlace(double & v, double vMin, double vMax)
{
  if (v > vMax)
  {
    v = vMax;
  }
  else if (v < vMin)
  {
    v = vMin;
  }
}

/** Clamp a value in a given interval, issuing a warning when bounds are hit.
 *
 * \param v Value.
 *
 * \param vMin Lower bound.
 *
 * \param vMax Upper bound.
 *
 * \param label Name of clamped value.
 *
 */
inline double clamp(double v, double vMin, double vMax, const char * label)
{
  if (v > vMax)
  {
    LOG_WARNING(label << " clamped to " << vMax);
    return vMax;
  }
  else if (v < vMin)
  {
    LOG_WARNING(label << " clamped to " << vMin);
    return vMin;
  }
  else
  {
    return v;
  }
}

/** Clamp a value in a given interval, issuing a warning when bounds are hit.
 *
 * \param v Reference to value.
 *
 * \param vMin Lower bound.
 *
 * \param vMax Upper bound.
 *
 * \param label Name of clamped value.
 *
 */
inline void clampInPlace(double & v, double vMin, double vMax, const char * label)
{
  if (v > vMax)
  {
    LOG_WARNING(label << " clamped to " << vMax);
    v = vMax;
  }
  else if (v < vMin)
  {
    LOG_WARNING(label << " clamped to " << vMin);
    v = vMin;
  }
}
