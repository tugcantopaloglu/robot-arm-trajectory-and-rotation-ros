cmake_minimum_required(VERSION 3.0.2)
project(ur10_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur10_kinematics
  CATKIN_DEPENDS roscpp std_msgs nav_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(ur10_kinematics
  src/kinematics.cpp
)

target_link_libraries(ur10_kinematics
  ${catkin_LIBRARIES}
)

add_executable(ur10_kinematics_node src/ur10_kinematics_node.cpp)
target_link_libraries(ur10_kinematics_node
  ${catkin_LIBRARIES}
  ur10_kinematics
)

add_executable(ur10_trajectory_node src/ur10_trajectory_node.cpp)
target_link_libraries(ur10_trajectory_node
  ${catkin_LIBRARIES}
  ur10_kinematics
)
