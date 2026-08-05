cmake_minimum_required(VERSION 3.0.2)
project(raicom_supervisor)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/raicom_supervisor_node.py
  scripts/raicom_acceptance_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/start_competition_task.sh
  scripts/provincial_preflight.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
