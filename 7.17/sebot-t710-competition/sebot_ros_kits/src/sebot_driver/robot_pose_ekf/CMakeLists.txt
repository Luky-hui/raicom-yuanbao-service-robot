cmake_minimum_required(VERSION 3.0.2)
project(robot_pose_ekf)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED
            COMPONENTS
            roscpp
            tf
            nav_msgs
            std_msgs
            geometry_msgs
            sensor_msgs
            message_generation
        )

find_package(Boost REQUIRED COMPONENTS thread)

# services
add_service_files(
    DIRECTORY srv
    FILES
    GetStatus.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        nav_msgs
        roscpp
        sensor_msgs
        std_msgs
)

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_executable(robot_pose_ekf 
            src/odom_estimation.cpp 
            src/nonlinearanalyticconditionalgaussianodo.cpp 
            src/odom_estimation_node.cpp)
target_link_libraries(robot_pose_ekf
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${BFL_LIBRARIES}
    )
add_dependencies(robot_pose_ekf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
    robot_pose_ekf
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES robot_pose_ekf.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
