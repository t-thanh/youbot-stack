cmake_minimum_required(VERSION 2.4.6)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
PROJECT(YouBot_deployment)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()

