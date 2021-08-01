cmake_minimum_required(VERSION 3.15)
project(velocity_manager)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  husarion_msgs
)


catkin_package(
 LIBRARIES velocity_manager
 CATKIN_DEPENDS roscpp rospy message_runtime husarion_msgs
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
  src/velocity_manager_node.cxx 
  src/VelocityManager.cxx
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
