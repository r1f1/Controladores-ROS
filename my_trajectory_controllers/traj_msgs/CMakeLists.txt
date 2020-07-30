cmake_minimum_required(VERSION 2.8.3)
project(traj_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)

add_message_files(
  FILES
  command_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()