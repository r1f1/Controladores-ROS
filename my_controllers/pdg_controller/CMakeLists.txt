cmake_minimum_required(VERSION 2.8.3)
project(pdg_controller)

add_compile_options(-std=c++11)

list( INSERT CMAKE_MODULE_PATH 0 "~/catkin_ws/src/my_controllers/cmake" )

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  std_msgs
)

find_package (RBDL COMPONENTS URDFReader REQUIRED)

catkin_package()

find_package (Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

add_library(pdg_controller_lib 
  src/pdg_controller.cpp
)

target_link_libraries(pdg_controller_lib
  ${catkin_LIBRARIES}
	${RBDL_LIBRARY}
	${RBDL_URDFReader_LIBRARY}
 )