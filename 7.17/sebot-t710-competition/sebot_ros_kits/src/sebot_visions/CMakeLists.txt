cmake_minimum_required(VERSION 3.0.2)
project(sebot_visions)
find_package(catkin REQUIRED COMPONENTS
)


catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include

  ${catkin_INCLUDE_DIRS}
)