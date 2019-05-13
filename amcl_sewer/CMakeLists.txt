cmake_minimum_required(VERSION 2.8.3)
project(amcl_sewer)

set (CMAKE_BUILD_TYPE Release)

set (CMAKE_CXX_FLAGS -std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cmake_modules
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions 
  pcl_ros
  sewer_graph
  simple_graph
  visualization_msgs
  nav_msgs
  plane_detector
  wall_detector
  message_generation
)

# GSL package
find_package(GSL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Localization.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_processing
 CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros nav_msgs tf sewer_graph simple_graph plane_detector message_runtime message_generation sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gsl_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(stereo_processing
#   src/${PROJECT_NAME}/stereo_processing.cpp
# )

## Declare a cpp executable
add_executable(amcl_node src/amcl_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(amcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(amcl_node
   ${catkin_LIBRARIES}
   ${GSL_LIBRARIES}
)
