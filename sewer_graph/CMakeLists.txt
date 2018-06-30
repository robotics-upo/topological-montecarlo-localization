project(sewer_graph)
cmake_minimum_required(VERSION 2.6)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp functions visualization_msgs roscpp geometry_msgs simple_graph)
# find_package(Boost REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES sewer_graph kmlbase kmldom kmlengine kmlconvenience dxflib geotranz
 CATKIN_DEPENDS roscpp functions visualization_msgs roscpp geometry_msgs simple_graph
 DEPENDS Boost 
)

# Check dependencies for kml libraries
find_library(KMLBASE_LIB NAMES kmlbase)
IF(${KMLBASE_LIB} STREQUAL "KMLBASE_LIB-NOTFOUND")
  message(FATAL_ERROR "kmlbase library not found!\nTry 'sudo apt-get install libkml-dev'\n")
ELSE()
  message(STATUS "Found kmlbase lib = ${KMLBASE_LIB}")
ENDIF()
find_library(KMLDOM_LIB NAMES kmldom)
IF(${KMLDOM_LIB} STREQUAL "KMLDOM_LIB-NOTFOUND")
  message(FATAL_ERROR "kmldom library not found!\nTry 'sudo apt-get install libkml-dev'\n")
ELSE()
  message(STATUS "Found kmldom lib = ${KMLDOM_LIB}")
ENDIF()
find_library(KMLENGINE_LIB NAMES kmlengine)
IF(${KMLENGINE_LIB} STREQUAL "KMLENGINE_LIB-NOTFOUND")
  message(FATAL_ERROR "kmlengine library not found!\nTry 'sudo apt-get install libkml-dev'\n")
ELSE()
  message(STATUS "Found kmlengine lib = ${KMLENGINE_LIB}")
ENDIF()
find_library(KMLCONV_LIB NAMES kmlconvenience)
IF(${KMLCONV_LIB} STREQUAL "KMLCONV_LIB-NOTFOUND")
  message(FATAL_ERROR "kmlconvenience library not found!\nTry 'sudo apt-get install libkml-dev'\n")
ELSE()
  message(STATUS "Found kmlconvenience lib = ${KMLCONV_LIB}")
ENDIF()

# Check dependencies for dxflib library
find_library(DXFLIB_LIB NAMES dxflib)
IF(${DXFLIB_LIB} STREQUAL "DXFLIB_LIB-NOTFOUND")
  message(FATAL_ERROR "dxflib library not found!\nTry 'sudo apt-get install libdxflib-dev'\n")
ELSE()
  message(STATUS "Found dxflib lib = ${DXFLIB_LIB}")
ENDIF()

# Check dependencies for geotranz library
find_library(GEOTRANZ_LIB NAMES geotranz)
IF(${GEOTRANZ_LIB} STREQUAL "GEOTRANZ_LIB-NOTFOUND")
  message(FATAL_ERROR "geotranz library not found!\nTry 'sudo apt-get install libgeotranz3-dev'\n")
ELSE()
  message(STATUS "Found geotranz lib = ${GEOTRANZ_LIB}")
ENDIF()

set(geotranz_include /usr/include/geotranz/dtcc/CoordinateSystems/ /usr/include/CoordinateSystemParameters/ 
/usr/include/CoordinateTuples/ /usr/include/geotranz/dtcc/CoordinateSystems/misc/ /usr/include/geotranz/dtcc
/usr/include/geotranz/dtcc/Enumerations)

include_directories(include ${catkin_INCLUDE_DIRS} ${geotranz_include} )

SET(FP_SOURCE src/earthlocation.cpp src/general.cpp src/sewer_graph.cpp) 
SET(LIBS ${catkin_LIBRARIES} kmlbase kmldom kmlengine kmlconvenience dxflib geotranz)

add_library(sewer_graph ${FP_SOURCE}
)
target_link_libraries(sewer_graph ${LIBS})


# Test Earth Location
 add_executable(test_earth_location src/exec/test_earth_location.cpp)
 target_link_libraries(test_earth_location ${PROJECT_NAME} ${LIBS})
 
# Test sewer graph
add_executable(test_sewer_graph src/exec/test_sewer_graph.cpp)
target_link_libraries(test_sewer_graph ${PROJECT_NAME} ${LIBS})

add_executable(dxf_to_sewer_graph src/exec/dxf_to_sewer_graph.cpp src/test_creationclass.cpp)
target_link_libraries(dxf_to_sewer_graph ${PROJECT_NAME} ${LIBS})

add_executable(get_results src/exec/get_results.cpp)
target_link_libraries(get_results ${PROJECT_NAME} ${LIBS})
 
 ## Mark executables and/or libraries for installation
install(TARGETS sewer_graph
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
