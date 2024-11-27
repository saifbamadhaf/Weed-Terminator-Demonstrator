#!/usr/bin/bash
export ROS_DOMAIN_ID=66
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
echo $ROS_DOMAIN_ID
ros2 launch cartesian_positioner_controller all_control.launch.py manual:=true $1
#sleep 10