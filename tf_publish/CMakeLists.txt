cmake_minimum_required(VERSION 2.8.3)
project(tf_publish)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # cv_bridge
  # image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  message_generation
  nav_msgs
)


catkin_package(
   CATKIN_DEPENDS message_runtime
   geometry_msgs
   nav_msgs
   std_msgs
   tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(tf_publish src/tf_publish.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(tf_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(tf_publish
  ${catkin_LIBRARIES}
)
