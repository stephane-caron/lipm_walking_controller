# Copyright (c) 2018-2019, CNRS-UM LIRMM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings -Wconversion")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME lipm_walking_controller)
set(PROJECT_DESCRIPTION "Walking controller based on linear inverted pendulum tracking")
set(PROJECT_URL https://github.com/stephane-caron/lipm_walking_controller/)
set(PROJECT_VERSION 1.3.0)

project(${PROJECT_NAME})  # because setup_project() is not called

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(copra REQUIRED)
find_package(mc_rtc REQUIRED)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp roslib std_msgs tf)

include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/.local/include)
link_directories(${catkin_LIBRARY_DIRS} $ENV{HOME}/.local/lib)

add_subdirectory(src)
