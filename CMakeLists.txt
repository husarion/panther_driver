cmake_minimum_required(VERSION 2.8.3)
project(panther_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy
)

add_compile_options(-std=c++11)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES panther_driver
 CATKIN_DEPENDS roscpp rospy
# DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/driver
  src/driver.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  launch/driver.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
