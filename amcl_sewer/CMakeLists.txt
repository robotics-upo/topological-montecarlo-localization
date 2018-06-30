cmake_minimum_required(VERSION 2.8.3)
project(amcl_sewer)

set (CMAKE_BUILD_TYPE Debug)

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
  geometry_msgs
  pcl_conversions 
  pcl_ros
  sewer_graph
  simple_graph
  visualization_msgs
  nav_msgs
  plane_detector
)

# GSL package
find_package( PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_processing
 CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros nav_msgs tf sewer_graph simple_graph gsll
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
add_dependencies(amcl_node plane_detector_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(amcl_node
   ${catkin_LIBRARIES}
   ${gsl_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS stereo_processing stereo_processing_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_stereo_processing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
