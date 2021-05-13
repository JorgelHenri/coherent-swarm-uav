cmake_minimum_required(VERSION 2.8.3)
project(coherent-swarm-uav-default)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
)
# include Eigen3
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES CoherentD
  CATKIN_DEPENDS roscpp std_msgs nav_msgs mrs_msgs mrs_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(CoherentD src/CoherentD.cpp)
add_dependencies(CoherentD ${catkin_EXPORTED_TARGETS})

target_link_libraries(CoherentD
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)