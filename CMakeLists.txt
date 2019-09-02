cmake_minimum_required(VERSION 2.8.3)
project(robotx_joy_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  usv_control_msgs
  rostate_machine
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robotx_joy_control
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs usv_control_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(robotx_joy_control_node src/robotx_joy_control_node.cpp src/robotx_joy_control.cpp)
target_link_libraries(robotx_joy_control_node ${catkin_LIBRARIES})
add_dependencies(robotx_joy_control_node usv_control_msgs_generate_messages_cpp)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(TARGETS robotx_joy_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)