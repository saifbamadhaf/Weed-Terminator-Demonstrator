#!/usr/bin/env python3

import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, AXIS_STATE_FULL_CALIBRATION_SEQUENCE
from time import sleep
import os
import sys

#add current directory to be able to import launch_utils file
dir_path = os.path.dirname(os.path.realpath(__file__)) + "/../"
sys.path.append(dir_path)

from utils import connect_odrives

odrvXA, odrvYZ = connect_odrives()

print("XA axes:")
odrive.utils.dump_errors(odrvXA)

print("\nYZ axes:")
odrive.utils.dump_errors(odrvYZ)
sleep(30)
