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

#include <mc_rtc/logging.h>

/** Utility functions and classes.
 *
 */
namespace utils
{
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
}

using utils::clamp;
using utils::clampInPlace;
