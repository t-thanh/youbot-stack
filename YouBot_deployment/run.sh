#!/bin/bash

source /opt/ros_toolchain/setup.bash;
source /opt/orocos_toolchain_ros/env.sh;

export RTT_MOTION_CONTROL_MSGS_PATH=$(rospack find rtt_motion_control_msgs)/lib
export YOUBOT_ADAPTERS=$(rospack find YouBot_adapters)/lib/orocos
export YOUBOT_OODL_PATH=$(rospack find YouBot_OODL)/lib/orocos
export RTT_PATH=$(rospack find rtt)/install
export OCL_PATH=$(rospack find ocl)/lib/orocos

export RTT_COMPONENT_PATH=${RTT_PATH}:${OCL_PATH}:${RTT_MOTION_CONTROL_MSGS_PATH}:${YOUBOT_OODL_PATH}:${YOUBOT_ADAPTERS}

rosrun ocl deployer-gnulinux -s $1 $2 $3 $4 $5
