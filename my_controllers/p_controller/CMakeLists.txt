cmake_minimum_required(VERSION 2.8.3)
project(p_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(p_controller_lib 
  src/p_controller.cpp
)

target_link_libraries(p_controller_lib
  ${catkin_LIBRARIES}
 )