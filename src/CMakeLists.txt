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

set(CONTROLLER_SRC
    Controller.cpp
    FloatingBaseObserver.cpp
    FootstepPlan.cpp
    ModelPredictiveControl.cpp
    NetWrenchObserver.cpp
    Pendulum.cpp
    Stabilizer.cpp
    SwingFoot.cpp)

set(CONTROLLER_HDR
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Contact.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/FloatingBaseObserver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/FootstepPlan.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/ModelPredictiveControl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/NetWrenchObserver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Pendulum.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Preview.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Sole.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Stabilizer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/State.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/SwingFoot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/LeakyIntegrator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/LowPassVelocityFilter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/clamp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/polynomials.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/rotations.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/stats.h)

set(CONTROLLER_NAME "${PROJET_NAME}_controller")

add_library(${PROJECT_NAME} SHARED ${CONTROLLER_SRC} ${CONTROLLER_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS")
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} copra::copra eigen-lssol::eigen-lssol mc_rtc::mc_control_fsm)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)

add_library(${CONTROLLER_NAME} SHARED lib.cpp)
set_target_properties(${CONTROLLER_NAME}
  PROPERTIES
    PREFIX ""
    OUTPUT_NAME "${PROJECT_NAME}"
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}")
target_link_libraries(${CONTROLLER_NAME} ${PROJECT_NAME})
install(TARGETS ${CONTROLLER_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)

set(CONF_OUT "$ENV{HOME}/.config/mc_rtc/controllers/LIPMWalking.conf")
set(AROBASE "@")
set(LIPM_WALKING_STATES_DIR "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/states/")
set(LIPM_WALKING_STATES_DATA_DIR "${LIPM_WALKING_STATES_DIR}/data")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../etc/LIPMWalking.conf.cmake" "${CONF_OUT}")
unset(AROBASE)

set(LIPM_WALKING_STATES_DIR "${MC_RTC_LIBDIR}/mc_controller/${PROJECT_NAME}/states")
set(LIPM_WALKING_STATES_DATA_DIR "${LIPM_WALKING_STATES_DIR}/data")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../etc/LIPMWalking.conf.cmake" "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMWalking.conf")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMWalking.conf"
    DESTINATION "${MC_RTC_LIBDIR}/mc_controller/etc/")

add_subdirectory(states)
