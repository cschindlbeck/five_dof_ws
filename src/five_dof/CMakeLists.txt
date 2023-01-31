cmake_minimum_required(VERSION 3.0.2)
project(five_dof)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  nodes/simple_moveit
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
