cmake_minimum_required(VERSION 2.8.3)
project(panther_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy
)

add_compile_options(-std=c++11)

catkin_package(
# INCLUDE_DIRS include
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
  src/ClassicKinematics.py
  src/MixedKinematics.py
  src/driver_node.py
  src/MecanumKinematics.py 
  src/PantherKinematics.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  launch/driver.launch
  launch/rap.launch
  launch/webui.launch
  launch/joy_mecanum.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  params/joystick.yaml
  params/panther.yaml
  params/joy_teleop.yaml
  params/roboteq_motor_controllers_v60.eds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)
