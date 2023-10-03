cmake_minimum_required(VERSION 3.5)
project(tiago_social_scenarios_sim)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  hubero_ros
)

## catkin specific configuration
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${hubero_ros_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS hubero_ros
)

## executables
add_executable(aws_hospital_normal_hubero src/aws_hospital_normal_hubero.cpp)
target_link_libraries(aws_hospital_normal_hubero
  ${hubero_ros_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(lab_012_dynamic_hubero src/lab_012_dynamic_hubero.cpp)
target_link_libraries(lab_012_dynamic_hubero
  ${hubero_ros_LIBRARIES}
  ${catkin_LIBRARIES}
)
