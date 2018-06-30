cmake_minimum_required(VERSION 2.8.3)
project(simple_graph)


find_package(catkin)
## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

catkin_package(
 INCLUDE_DIRS include include/simple_graph
 LIBRARIES functions
)

include_directories(${CATKIN_INCLUDE_DIRS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

add_executable(test src/test.cpp)