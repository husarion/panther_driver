# build with <catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release>
cmake_minimum_required(VERSION 3.8)
project(panther_driver)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  geometry_msgs
  message_generation
  husarion_msgs
)


catkin_package(
# INCLUDE_DIRS include
 LIBRARIES panther_driver
 CATKIN_DEPENDS roscpp rospy message_runtime husarion_msgs
# DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(Headers
  include/VelocityManager.hpp
  include/StateMachine.hpp
)

set(Sources
  src/panther_driver/velocity_manager_node.cxx 
  src/panther_driver/VelocityManager.cxx
)

add_executable(velocity_manager_node ${Sources} )
add_dependencies(velocity_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_property(TARGET velocity_manager_node PROPERTY CXX_STANDARD 17)
set_property(TARGET velocity_manager_node PROPERTY CXX_STANDARD_REQUIRED ON)

if(CATKIN_ENABLE_TESTING)
  add_library(${This} STATIC ${Sources} ${Headers})
  add_subdirectory(test)
endif()

target_link_libraries(velocity_manager_node
  ${catkin_LIBRARIES} 
)