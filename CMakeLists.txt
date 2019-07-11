# Copyright 2018-2019 CNRS-UM LIRMM
#
# This file is part of lipm_walking_controller.
#
# lipm_walking_controller is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# lipm_walking_controller is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
# General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with lipm_walking_controller. If not, see
# <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8)

#set(ROS_BUILD_TYPE Debug) # remove for Release
#set(CXX_DISABLE_WERROR 1)

set(CMAKE_CXX_FLAGS "-pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings -Wconversion")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME lipm_walking_controller)
set(PROJECT_DESCRIPTION "Walking controller based on linear inverted pendulum tracking")
set(PROJECT_URL https://github.com/stephane-caron/lipm_walking_controller/)
set(PROJECT_VERSION 1.1.0)

project(${PROJECT_NAME})  # because setup_project() is not called

# Enable C++14
if(NOT WIN32)
  add_definitions(-std=c++14)
endif()

# Use jrl-cmakemodules to get mc_rtc depency
set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
add_required_dependency(copra)
add_compile_options(-DEIGEN_QLD_FOUND) # see https://github.com/vsamy/copra/issues/2
add_compile_options(-DEIGEN_LSSOL_FOUND) # see https://github.com/vsamy/copra/issues/2
add_required_dependency(eigen-lssol)
add_required_dependency(mc_rtc)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  roslib
  std_msgs
  tf
)

include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/.local/include)
link_directories(${catkin_LIBRARY_DIRS} $ENV{HOME}/.local/lib)

add_subdirectory(src)
