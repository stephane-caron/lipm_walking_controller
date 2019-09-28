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

set(FSM_STATES_INSTALL_PREFIX "${MC_RTC_LIBDIR}/mc_controller/${PROJECT_NAME}/states")

macro(add_fsm_state state_name state_SRC state_HDR)
  add_library(${state_name} SHARED  ${state_SRC} ${state_HDR})
  set_target_properties(${state_name}
      PROPERTIES
      PREFIX ""
      LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/states")
  target_link_libraries(${state_name} ${PROJECT_NAME})
  install(TARGETS ${state_name} DESTINATION ${FSM_STATES_INSTALL_PREFIX})
  install(FILES ${state_HDR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/mc_control/fsm/states)
endmacro()

# install(DIRECTORY data DESTINATION ${FSM_STATES_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.json")

macro(add_fsm_state_simple state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${state_name}.h)
endmacro()

add_fsm_state_simple(DoubleSupport)
add_fsm_state_simple(Initial)
add_fsm_state_simple(SingleSupport)
add_fsm_state_simple(Standing)
