cmake_minimum_required(VERSION 3.0.2)
project(onrobot_rg_joint_position_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
  CATKIN_DEPENDS
  rospy
  onrobot_rg_control
)

catkin_install_python(
  PROGRAMS
  scripts/OnRobotRGJointPositionControllerServer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})