cmake_minimum_required(VERSION 2.6.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE MinSizeRel)

rosbuild_init()

rosbuild_genmsg()

rosbuild_include(rtt_rosnode GenerateRTTtypekit)

ros_generate_rtt_typekit(joy)

