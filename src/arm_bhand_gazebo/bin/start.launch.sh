#!/bin/bash

source $(dirname $(realpath $0))/../../../devel/setup.bash

/opt/ros/kinetic/bin/roslaunch arm_bhand_gazebo start.launch
