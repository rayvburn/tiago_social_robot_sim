cmake_minimum_required(VERSION 3.5)
project(tiago_social_bringup_sim)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  image_proc
  pal_navigation_sm
  pal_gazebo_worlds
  tiago_gazebo
  tiago_laser_sensors
)

## catkin specific configuration
# No packages marked as CATKIN_DEPENDS since this package does not require compilation
catkin_package()

## Install
# ref: https://github.com/pal-robotics/tiago_robot/blob/kinetic-devel/tiago_bringup/CMakeLists.txt
foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
