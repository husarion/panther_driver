cmake_minimum_required(VERSION 2.8.3)
project(panther_driver)

find_package(Threads REQUIRED)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
)


install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
