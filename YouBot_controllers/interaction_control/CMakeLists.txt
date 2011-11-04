CMAKE_MINIMUM_REQUIRED(VERSION 2.6.3)
project(ModelControl_interaction_control)

#
# Output structure
#
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

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
 
   #  Set the build type.  Options are:
   #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
   #  Debug          : w/ debug symbols, w/o optimization
   #  Release        : w/o debug symbols, w/ optimization
   #  RelWithDebInfo : w/ debug symbols, w/ optimization
   #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
   set(ROS_BUILD_TYPE RelWithDebInfo)
else(ROS_ROOT)
   #Add necessary libraries
   orocos_use_package(ocl-taskbrowser)
   orocos_use_package(rtt)
endif(ROS_ROOT)  
# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
#Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
# Determine target
set(OROCOS_TARGET $ENV{OROCOS_TARGET})
IF(${OROCOS_TARGET} MATCHES xenomai)
   INCLUDE_DIRECTORIES(/usr/xenomai/include)
ENDIF(${OROCOS_TARGET} MATCHES xenomai)
#Add link directories for libraries
IF(${OROCOS_TARGET} MATCHES xenomai)
    LINK_DIRECTORIES(/usr/xenomai/lib)
    LINK_LIBRARIES(native)
ENDIF(${OROCOS_TARGET} MATCHES xenomai)

#Add include directories
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(include/common)

ADD_SUBDIRECTORY(src)
#ADD_SUBDIRECTORY(test)

#
orocos_generate_package()



