#!/usr/bin/bash

source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch cartesian_positioner_controller demo_mode_dynamic.launch.py $1