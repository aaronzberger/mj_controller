cmake_minimum_required(VERSION 3.0.2)
project(mj_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate services in the 'srv' folder
add_service_files(
  FILES
  RegisterGroup.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
 CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)
