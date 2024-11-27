#!/usr/bin/bash

source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
cd "$(dirname "$0")"
python3 homing_python.py