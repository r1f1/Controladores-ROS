cmake_minimum_required(VERSION 2.8.3)
project(pd_pid_traj_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  traj_msgs
)

catkin_package(CATKIN_DEPENDS traj_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(pd_pid_traj_controller_lib 
  src/pd_pid_traj_controller.cpp
)

target_link_libraries(pd_pid_traj_controller_lib
  ${catkin_LIBRARIES}
 )