cmake_minimum_required(VERSION 2.8.3)
project(r_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  r_msgs
  diagnostic_msgs
  diagnostic_updater
)

find_package(Boost REQUIRED system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES r_lib
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf r_msgs r_description diagnostic_msgs diagnostic_updater
)

include(ExternalProject)
ExternalProject_Add(r_lib
  GIT_REPOSITORY https://github.com/AutonomyLab/libcreate.git
  GIT_TAG 1.3.0
  PREFIX ${CATKIN_DEVEL_PREFIX}
  CONFIGURE_COMMAND cmake .
  BUILD_COMMAND make
  INSTALL_COMMAND echo "No install"
  BUILD_IN_SOURCE 1
)

set(r_lib_PATH "${CATKIN_DEVEL_PREFIX}/src/r_lib/")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${r_lib_PATH}/include
  include
)

link_directories(${r_lib_PATH})

add_executable(r_driver src/r_driver.cpp)

target_link_libraries(r_driver
  create
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(r_driver r_lib)
