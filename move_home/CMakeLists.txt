cmake_minimum_required(VERSION 2.8.3)
project(move_home)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  traj_msgs
)

catkin_package(CATKIN_DEPENDS traj_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_home 
  src/move_home.cpp
)

add_executable(move_home_traj 
  src/move_home_traj.cpp
)

add_dependencies(move_home ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(move_home_traj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(move_home
  ${catkin_LIBRARIES}
)

target_link_libraries(move_home_traj
  ${catkin_LIBRARIES}
)