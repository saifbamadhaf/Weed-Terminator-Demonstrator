#!/usr/bin/env python3

import odrive
import os
import sys
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, AXIS_STATE_FULL_CALIBRATION_SEQUENCE
from time import sleep

#add current directory to be able to import launch_utils file
dir_path = os.path.dirname(os.path.realpath(__file__)) + "/../"
sys.path.append(dir_path)

from utils import connect_odrives

odrvXA, odrvYZ = connect_odrives()

# Set odrives to idle
odrvXA.axis0.requested_state = AXIS_STATE_IDLE
odrvXA.axis1.requested_state = AXIS_STATE_IDLE
odrvYZ.axis0.requested_state = AXIS_STATE_IDLE
odrvYZ.axis1.requested_state = AXIS_STATE_IDLE

odrvXA.clear_errors()
odrvYZ.clear_errors()
print("Done")

