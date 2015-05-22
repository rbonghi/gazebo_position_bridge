cmake_minimum_required(VERSION 2.8.3)
project(gazebo_position_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gazebo_position_bridge
#  CATKIN_DEPENDS gazebo roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(gazebo_position_bridge
#   src/${PROJECT_NAME}/gazebo_position_bridge.cpp
# )

## Declare a cpp executable
add_executable(gazebo_bridge_node src/gazebo_bridge_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(gazebo_bridge_node gazebo_bridge_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gazebo_bridge_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

