set(FSM_STATES_INSTALL_PREFIX "${MC_RTC_LIBDIR}/mc_controller/lipm_walking_controller/states")

macro(add_fsm_state state_name state_SRC state_HDR)
  add_library(${state_name} SHARED  ${state_SRC} ${state_HDR})
  set_target_properties(${state_name}
      PROPERTIES
      PREFIX ""
      LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/states")
  target_link_libraries(${state_name} lipm_walking_controller)
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
