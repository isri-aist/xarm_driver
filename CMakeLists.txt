cmake_minimum_required(VERSION 3.14)

set(PROJECT_NAME xarm_driver)

project(
  ${PROJECT_NAME}
  VERSION 0.1.0
  LANGUAGES CXX
)

include(GNUInstallDirs)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(unified_robot_interface REQUIRED)
find_package(fmt REQUIRED)
find_package(xarm REQUIRED)

add_subdirectory(src)
