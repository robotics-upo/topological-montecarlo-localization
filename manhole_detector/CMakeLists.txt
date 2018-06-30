cmake_minimum_required(VERSION 2.8.3)
project(manhole_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  rospy
  sensor_msgs
  std_msgs
  compressed_depth_image_transport
  cv_bridge
  dynamic_reconfigure
  image_transport
  functions
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manhole_labeler
 CATKIN_DEPENDS rosbag roscpp rospy sensor_msgs std_msgs compressed_depth_image_transport   cv_bridge   dynamic_reconfigure   image_transport functions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIRS} 
)

## Declare a C++ executable
add_executable(generate_dataset src/generate_dataset_learning.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(generate_dataset
  ${catkin_LIBRARIES}  ${Boost_LIBRARIES}
)



