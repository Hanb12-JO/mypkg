#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source $dir/.bashrc
timeout 40 ros2 run assignment2 wifispeed.py | tee - /tmp/assignment2.log


cat /tmp/assingnment2.log |
grep ''
