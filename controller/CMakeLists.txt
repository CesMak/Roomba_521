cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  turtlesim
std_msgs
)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

#Diese Zeilen sind sehr wichtig ansonsten wird kann dieser Code hier nicht als node in einem
#anderen package verwendet werden!
add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
