#!/bin/bash

rxplot -b 10 /debug/joint_states/position[0]:position[1]:position[2]:position[3]:position[4]&
rxplot -b 10 /debug/joint_cmd_angles/positions[0]:positions[1]:positions[2]:positions[3]:positions[4]&
rostopic echo /debug/events

