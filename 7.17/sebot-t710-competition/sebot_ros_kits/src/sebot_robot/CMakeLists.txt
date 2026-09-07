cmake_minimum_required(VERSION 2.8.3)
project(sebot_robot)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs
)

find_package(PkgConfig)
pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14) 

add_executable(sebot_controller src/controller.cpp)
add_dependencies(sebot_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sebot_controller ${catkin_LIBRARIES} ${SERIAL_LIBRARIES})

add_executable(sebot_joystick src/joystick.cpp)
target_link_libraries(sebot_joystick ${catkin_LIBRARIES})

add_executable(sebot_keyboard src/keyboards.cpp)
target_link_libraries(sebot_keyboard ${catkin_LIBRARIES})

add_executable(sebot_transform src/transform.cpp)
target_link_libraries(sebot_transform ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

