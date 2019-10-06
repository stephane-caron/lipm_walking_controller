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

set(CMAKE_CXX_FLAGS "-pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings -Wconversion")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME lipm_walking_controller)
set(PROJECT_DESCRIPTION "Walking controller based on linear inverted pendulum tracking")
set(PROJECT_URL https://github.com/stephane-caron/lipm_walking_controller/)
set(PROJECT_VERSION 1.1.0)

project(${PROJECT_NAME})  # because setup_project() is not called

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(copra REQUIRED)
find_package(mc_rtc REQUIRED)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp roslib std_msgs tf)

include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/.local/include)
link_directories(${catkin_LIBRARY_DIRS} $ENV{HOME}/.local/lib)

add_subdirectory(src)
