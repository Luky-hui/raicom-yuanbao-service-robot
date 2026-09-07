cmake_minimum_required(VERSION 3.0.2)
project(sebot_talon)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  rospy
  # moveit_ros_planning_interface
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sebot_talon
#  CATKIN_DEPENDS urdf xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


