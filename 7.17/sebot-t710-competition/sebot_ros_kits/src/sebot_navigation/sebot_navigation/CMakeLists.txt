cmake_minimum_required(VERSION 3.0.2)
project(sebot_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf
  actionlib
  actionlib_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs tf actionlib actionlib_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(multinavi 
  src/multinavi.cpp 
  src/tinystr.cpp 
  src/tinyxml.cpp 
  src/tinyxmlerror.cpp 
  src/tinyxmlparser.cpp) 
target_link_libraries(multinavi ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")