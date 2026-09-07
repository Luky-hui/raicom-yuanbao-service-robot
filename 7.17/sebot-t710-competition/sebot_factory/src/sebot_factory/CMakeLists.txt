cmake_minimum_required(VERSION 2.8.3)
project(sebot_factory)

set(CMAKE_CXX_STANDARD 14) 
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  roslib
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs actionlib actionlib_msgs
)

find_package(PkgConfig)
pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)

# 智能工厂
add_executable(sebot_factory ${PROJECT_SOURCE_DIR}/src/factory.cpp ${PROJECT_SOURCE_DIR}/include/tinyxml.hpp)
add_dependencies(sebot_factory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sebot_factory ${catkin_LIBRARIES} ${SERIAL_LIBRARIES} ${PPNC_LIBRARIES} ${ONNX_LIBRARIES} ${OpenCV_LIBS})

# 单元测试 [AI目标检测]
add_executable(sebot_yolo ${PROJECT_SOURCE_DIR}/unit/yolo.cpp)
add_dependencies(sebot_yolo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sebot_yolo ${catkin_LIBRARIES} ${PPNC_LIBRARIES} ${ONNX_LIBRARIES} ${OpenCV_LIBS})

# # 单元测试 [需求确认]
# add_executable(sebot_confirm ${PROJECT_SOURCE_DIR}/unit/order.cpp)
# add_dependencies(sebot_confirm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(sebot_confirm ${catkin_LIBRARIES} ${PPNC_LIBRARIES} ${ONNX_LIBRARIES} ${OpenCV_LIBS})

# # 单元测试 [智能取件]
# add_executable(sebot_picking ${PROJECT_SOURCE_DIR}/unit/pick.cpp)
# add_dependencies(sebot_picking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(sebot_picking ${catkin_LIBRARIES} ${SERIAL_LIBRARIES} ${PPNC_LIBRARIES} ${ONNX_LIBRARIES} ${OpenCV_LIBS})

# 单元测试 [智能收银]
add_executable(sebot_summary ${PROJECT_SOURCE_DIR}/unit/pay.cpp)
add_dependencies(sebot_summary ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sebot_summary ${catkin_LIBRARIES} ${PPNC_LIBRARIES} ${ONNX_LIBRARIES} ${OpenCV_LIBS})

# 单元测试 [机械臂]
add_executable(sebot_arm ${PROJECT_SOURCE_DIR}/unit/arm.cpp)
add_dependencies(sebot_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sebot_arm ${catkin_LIBRARIES} ${SERIAL_LIBRARIES})
