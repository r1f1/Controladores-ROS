cmake_minimum_required(VERSION 2.8.3)
project(def_pos)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  traj_msgs
)

catkin_package(CATKIN_DEPENDS traj_msgs)

add_definitions( -O2)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(def_pos 
  src/def_pos.cpp
  src/Trayectoria2.cpp
)

add_dependencies(def_pos ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(def_pos
  ${catkin_LIBRARIES}
  armadillo
)

