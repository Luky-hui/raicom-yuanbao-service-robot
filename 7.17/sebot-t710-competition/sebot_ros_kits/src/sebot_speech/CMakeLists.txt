cmake_minimum_required(VERSION 2.8.3)
project(sebot_speech)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# 查找ALSA库  
find_package(PkgConfig REQUIRED)  
pkg_check_modules(ALSA REQUIRED alsa)  

# 添加头文件搜索路径  
include_directories(${ALSA_INCLUDE_DIRS})  

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14) 

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

