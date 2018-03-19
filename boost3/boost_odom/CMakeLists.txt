cmake_minimum_required(VERSION 2.8.3)
project(boost_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  urdf
  genmsg
)


################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

## Declare a C++ executable
add_executable(boost_odom_pub src/boost_odom_pub.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(boost_odom_pub
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
