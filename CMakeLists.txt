cmake_minimum_required(VERSION 2.8)

#set(ROS_BUILD_TYPE Debug) # remove for Release
#set(CXX_DISABLE_WERROR 1)

set(CMAKE_CXX_FLAGS "-pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings -Wconversion")

include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME lipm_walking_controller)
set(PROJECT_DESCRIPTION "Walking controller based on linear inverted pendulum tracking")
set(PROJECT_URL https://github.com/stephane-caron/lipm_walking_controller/)
set(PROJECT_VERSION 1.0.0)

# setup_project()  # remove for catkin

project(${PROJECT_NAME})  # because setup_project() is not called

# Enable C++11
#if(NOT WIN32)
#  add_definitions(-std=c++0x)
#endif()
# Enable C++14
if(NOT WIN32)
  add_definitions(-std=c++14)
endif()

# Use jrl-cmakemodules to get mc_rtc depency
set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
add_required_dependency(mc_rtc)

find_package(catkin REQUIRED)

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

# setup_project_finalize()  # removed for catkin
