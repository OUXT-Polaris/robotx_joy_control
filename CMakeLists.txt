cmake_minimum_required(VERSION 2.8.3)
project(vrx_joystick)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vrx_joystick
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(joy_operator_node src/joy_operator.cpp src/joy_operator_node.cpp)
target_link_libraries(joy_operator_node ${catkin_LIBRARIES})