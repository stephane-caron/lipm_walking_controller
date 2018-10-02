set(CONTROLLER_SRC
    Controller.cpp
    FloatingBaseObserver.cpp
    FootstepPlan.cpp
    HorizontalMPCProblem.cpp
    HorizontalMPCSolution.cpp
    Pendulum.cpp
    PendulumObserver.cpp
    Python.cpp
    Stabilizer.cpp
    SwingFoot.cpp
    ros.cpp
)

set(CONTROLLER_HDR
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Contact.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/FloatingBaseObserver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/FootstepPlan.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/HorizontalMPC.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/HorizontalMPCProblem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/HorizontalMPCSolution.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Pendulum.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/PendulumObserver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Sole.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/Stabilizer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/State.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/SwingFoot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/AvgStdEstimator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/Integrator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/VelocityFilter.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/clamp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/polynomials.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/ros.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/lipm_walking/utils/rotations.h
)

set(FSM_CONTROLLER_LIB mc_control_fsm)
add_library(${PROJECT_NAME} SHARED ${CONTROLLER_SRC} ${CONTROLLER_HDR})
set_target_properties(${PROJECT_NAME}
    PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
    PREFIX ""
    OUTPUT_NAME "${PROJECT_NAME}"
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}")
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} roslib ${FSM_CONTROLLER_LIB} copra eigen-cddlib)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)

set(AROBASE "@")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../etc/LIPMWalking.conf.cmake ${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMWalking.conf)
unset(AROBASE)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMWalking.conf
    DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

add_subdirectory(states)
