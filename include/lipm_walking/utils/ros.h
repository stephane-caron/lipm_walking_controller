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

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

/** Point from 3D vector.
 *
 * \param vec Vector.
 *
 */
inline geometry_msgs::Point rosPoint(const Eigen::Vector3d & vec)
{
  geometry_msgs::Point p;
  p.x = vec.x();
  p.y = vec.y();
  p.z = vec.z();
  return p;
}

/** TF from Plucker transform.
 *
 * \param X Plucker transform.
 *
 * \param tm Time.
 *
 * \param from Name of parent frame.
 *
 * \param to Name of child frame.
 *
 * \param seq Sequence number.
 *
 */
inline geometry_msgs::TransformStamped PT2TF(const sva::PTransformd & X, const ros::Time & tm, const std::string & from, const std::string & to, unsigned int seq)
{
  Eigen::Vector4d q = Eigen::Quaterniond(X.rotation()).inverse().coeffs();
  const Eigen::Vector3d & t = X.translation();
  geometry_msgs::TransformStamped msg;
  msg.header.seq = seq;
  msg.header.stamp = tm;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = t.x();
  msg.transform.translation.y = t.y();
  msg.transform.translation.z = t.z();
  msg.transform.rotation.w = q.w();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  return msg;
}
