cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(YouBot_executive)

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
  rosbuild_include(rtt_rosnode GenerateRTTtypekit)

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
#include(/opt/eclipse/local.cmake)
# Determine target
set(OROCOS_TARGET $ENV{OROCOS_TARGET})

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/external/cmake_modules)
include(${CMAKE_CURRENT_SOURCE_DIR}/../eclipse.cmake)

#
# Compile stuff
#


SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin)
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

#defines the core sources and includes
SET(COMMON_INCLUDE_DIR ${CMAKE_HOME_DIRECTORY}/include)

SET(CMAKE_BUILD_TYPE Debug)

#include all CMakeList.txt files within this package
#each generates one library or application	
ADD_SUBDIRECTORY(src)

#ADD_SUBDIRECTORY(examples) 

orocos_generate_package()

endif()
