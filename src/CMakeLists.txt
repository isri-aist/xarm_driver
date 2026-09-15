if(NOT TARGET mc_robot_interface::robot_driver_api)
  message(
    FATAL_ERROR
      "Required target mc_robot_interface::robot_driver_api is unavailable"
  )
endif()

if(NOT TARGET xarm_sdk)
  message(FATAL_ERROR "Required local imported target xarm_sdk is unavailable")
endif()

add_library(RobotDriverxArm SHARED RobotDriverxArm.cpp)

target_compile_features(RobotDriverxArm PUBLIC cxx_std_17)

target_include_directories(
  RobotDriverxArm
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  RobotDriverxArm PUBLIC mc_robot_interface::robot_driver_api
)

target_link_libraries(RobotDriverxArm PRIVATE fmt::fmt xarm_sdk)

install(
  TARGETS RobotDriverxArm
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/robot_interface
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/robot_interface
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../include/xarm_driver"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)
