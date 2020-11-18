cmake_minimum_required(VERSION 2.8.3)
project(panther_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  tf
)

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

add_executable(docking_node src/docking_node.cpp src/DockingController.cpp)

target_link_libraries(docking_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch config params
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(TARGETS docking_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
