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

#include <cmath>

#include <mc_tasks/CoPTask.h>

#include <lipm_walking/defs.h>

namespace lipm_walking
{
  /** All four combinations of two foot contacts.
   *
   */
  enum class ContactState
  {
    DoubleSupport,
    LeftFoot,
    RightFoot,
    Flying
  };

  /** Contacts wrap foot frames with extra info from the footstep plan.
   *
   */
  struct Contact
  {
    /** Empty constructor.
     *
     * Plücker transform is left uninitialized.
     *
     */
    Contact()
      : refVel({0., 0., 0.}),
        halfLength(0.),
        halfWidth(0.),
        surfaceName(""),
        pose(),
        id(0)
    {
    }

    /** Constructor from Plücker transform.
     *
     * \param pose Plücker transform from inertial to contact frame.
     *
     */
    Contact(const sva::PTransformd & pose)
      : refVel({0., 0., 0.}),
        halfLength(0.),
        halfWidth(0.),
        surfaceName(""),
        pose(pose),
        id(0)
    {
    }

    /** Sagittal unit vector of the contact frame.
     *
     */
    Eigen::Vector3d sagittal() const
    {
      return pose.rotation().row(0);
    }

    /** Lateral unit vector of the contact frame.
     *
     */
    Eigen::Vector3d lateral() const
    {
      return pose.rotation().row(1);
    }

    /** Normal unit vector of the contact frame.
     *
     */
    Eigen::Vector3d normal() const
    {
      return pose.rotation().row(2);
    }

    /** World position of the contact frame.
     *
     */
    const Eigen::Vector3d & position() const
    {
      return pose.translation();
    }

    /** Shorthand for lateral vector.
     *
     */
    inline Eigen::Vector3d b() const
    {
      return lateral();
    }

    /** Shorthand for normal vector.
     *
     */
    inline Eigen::Vector3d n() const
    {
      return normal();
    }

    /** Shorthand for sagittal vector.
     *
     */
    inline Eigen::Vector3d t() const
    {
      return sagittal();
    }

    /** Shorthand for position.
     *
     */
    inline const Eigen::Vector3d & p() const
    {
      return position();
    }

    /** Position of ankle from foot center frame.
     *
     */
    inline Eigen::Vector3d anklePos() const
    {
      if (surfaceName == "LeftFootCenter")
      {
        return p() - 0.015 * t() - 0.01 * b();
      }
      else if (surfaceName == "RightFootCenter")
      {
        return p() - 0.015 * t() + 0.01 * b();
      }
      else
      {
        LOG_ERROR("Cannot compute anklePos for surface " << surfaceName);
        return p();
      }
    }

    /** Get frame rooted at the ankle.
     *
     */
    inline sva::PTransformd anklePose() const
    {
      return {pose.rotation(), anklePos()};
    }

    /** Shorthand for world x-coordinate.
     *
     */
    inline double x() const
    {
      return position()(0);
    }

    /** Shorthand for world y-coordinate.
     *
     */
    inline double y() const
    {
      return position()(1);
    }

    /** Shorthand for world z-coordinate.
     *
     */
    inline double z() const
    {
      return position()(2);
    }

    /** Corner vertex of the contact area.
     *
     */
    inline Eigen::Vector3d vertex0() const
    {
      return position() + halfLength * t() + halfWidth * b();
    }

    /** Corner vertex of the contact area.
     *
     */
    inline Eigen::Vector3d vertex1() const
    {
      return position() + halfLength * t() - halfWidth * b();
    }

    /** Corner vertex of the contact area.
     *
     */
    inline Eigen::Vector3d vertex2() const
    {
      return position() - halfLength * t() - halfWidth * b();
    }

    /** Corner vertex of the contact area.
     *
     */
    inline Eigen::Vector3d vertex3() const
    {
      return position() - halfLength * t() + halfWidth * b();
    }

    /** Minimum coordinate for vertices of the contact area.
     *
     */
    template <int i>
    inline double minCoord() const
    {
      return std::min(std::min(vertex0()(i), vertex1()(i)), std::min(vertex2()(i), vertex3()(i)));
    }

    /** Maximum coordinate for vertices of the contact area.
     *
     */
    template <int i>
    inline double maxCoord() const
    {
      return std::max(std::max(vertex0()(i), vertex1()(i)), std::max(vertex2()(i), vertex3()(i)));
    }

    /** Minimum world x-coordinate of the contact area.
     *
     */
    inline double xmin() const
    {
      return minCoord<0>();
    }

    /** Maximum world x-coordinate of the contact area.
     *
     */
    inline double xmax() const
    {
      return maxCoord<0>();
    }

    /** Minimum world y-coordinate of the contact area.
     *
     */
    inline double ymin() const
    {
      return minCoord<1>();
    }

    /** Maximum world y-coordinate of the contact area.
     *
     */
    inline double ymax() const
    {
      return maxCoord<1>();
    }

    /** Minimum world z-coordinate of the contact area.
     *
     */
    inline double zmin() const
    {
      return minCoord<2>();
    }

    /** Maximum world z-coordinate of the contact area.
     *
     */
    inline double zmax() const
    {
      return maxCoord<2>();
    }

    /** Move contact by a given magnitude in a random direction.
     *
     * \param magnitude Absolute displacement after noising.
     *
     */
    inline Contact addNoise(double magnitude) const
    {
      Contact noisedContact = *this;
      Eigen::Vector3d unitRandom = Eigen::Vector3d::Random().normalized();
      Eigen::Vector3d displacement = magnitude * unitRandom;
      noisedContact.pose = sva::PTransformd{displacement} * this->pose;
      return noisedContact;
    }

  public:
    Eigen::Vector3d refVel;
    bool pauseAfterSwing = false;
    double halfLength;
    double halfWidth;
    mc_rtc::Configuration swingConfig;
    std::string surfaceName;
    sva::PTransformd pose;
    unsigned id;
  };

  /** Apply Plucker transform to contact frame.
   *
   * \param X Transform to apply.
   *
   * \param contact Contact frame.
   *
   */
  inline Contact operator*(const sva::PTransformd & X, const Contact & contact)
  {
    Contact result = contact;
    result.pose = X * contact.pose;
    return result;
  }
}

namespace mc_rtc
{
  template<>
  struct ConfigurationLoader<lipm_walking::Contact>
  {
    static lipm_walking::Contact load(const mc_rtc::Configuration & config)
    {
      lipm_walking::Contact contact;
      contact.pose = config("pose");
      config("half_length", contact.halfLength);
      config("half_width", contact.halfWidth);
      config("ref_vel", contact.refVel);
      config("surface", contact.surfaceName);
      if (config.has("pause_after_swing"))
      {
        contact.pauseAfterSwing = config("pause_after_swing");
      }
      if (config.has("swing"))
      {
        contact.swingConfig = config("swing");
      }
      return contact;
    }

    static mc_rtc::Configuration save(const lipm_walking::Contact & contact)
    {
      mc_rtc::Configuration config;
      config.add("half_length", contact.halfLength);
      config.add("half_width", contact.halfWidth);
      config.add("pose", contact.pose);
      config.add("ref_vel", contact.refVel);
      config.add("surface", contact.surfaceName);
      if (contact.pauseAfterSwing)
      {
        config.add("pause_after_swing", true);
      }
      if (!contact.swingConfig.empty())
      {
        config("swing") = contact.swingConfig;
      }
      return config;
    }
  };
}
