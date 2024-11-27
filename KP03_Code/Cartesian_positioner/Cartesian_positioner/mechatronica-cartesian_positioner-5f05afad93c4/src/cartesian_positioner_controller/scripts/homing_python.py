#!/usr/bin/env python3

import os
import sys
import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, AXIS_STATE_ENCODER_INDEX_SEARCH
from time import sleep
import yaml

#add current directory to be able to import launch_utils file
dir_path = os.path.dirname(os.path.realpath(__file__)) + "/../"
sys.path.append(dir_path)

from utils import connect_odrives

odrvXA, odrvYZ = connect_odrives()

# Initialize variables
x_homed = False
y_homed = False
z_homed = False
moving = False
homing_speed = 1.2
stopped_tolerance = 0.04
moving_tolerance = homing_speed*0.75

odrvXA.clear_errors()
odrvYZ.clear_errors()

# Set odrives to calibrate encoder offset
odrvYZ.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrvYZ.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrvXA.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrvXA.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

# Wait for calibration
sleep(10)

# Make sure velocity setpoints are at 0
odrvXA.axis0.controller.input_vel = 0
odrvXA.axis1.controller.input_vel = 0
odrvYZ.axis0.controller.input_vel = 0
odrvYZ.axis1.controller.input_vel = 0

# Set odrives to velocity control
odrvXA.axis0.controller.config.control_mode = 2
odrvXA.axis1.controller.config.control_mode = 2
odrvYZ.axis0.controller.config.control_mode = 2
odrvYZ.axis1.controller.config.control_mode = 2

# Set odrives to closed loop control
odrvXA.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrvXA.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrvYZ.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrvYZ.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


# Move YZ axes away from home for 2s to ensure that homing is possible
print("Moving back YZ before homing")
odrvYZ.axis0.controller.input_vel = homing_speed
odrvYZ.axis1.controller.input_vel = 0 # note that YZ1 is set to 0 to make the axis move diagonally away from home

sleep(1)

odrvYZ.axis0.controller.input_vel = 0
odrvYZ.axis1.controller.input_vel = 0

sleep(2)

print("Homing Z")

# Homing YZ Loop
while(True):

    # Constantly update velocity estimates
    vel_yz0 = abs(odrvYZ.axis0.encoder.vel_estimate)
    vel_yz1 = abs(odrvYZ.axis1.encoder.vel_estimate)

    if (z_homed == False):
        odrvYZ.axis0.controller.input_vel = -homing_speed
        odrvYZ.axis1.controller.input_vel = homing_speed

        if (vel_yz0 > moving_tolerance and vel_yz1 > moving_tolerance):
            moving = True

        if (moving and (vel_yz0 + vel_yz1)/2 < stopped_tolerance):
            odrvYZ.axis0.controller.input_vel = 0
            odrvYZ.axis1.controller.input_vel = 0

            print("Z homed, homing Y")
            z_homed = True
            moving = False

    if (z_homed == True and y_homed == False):
        odrvYZ.axis0.controller.input_vel = -homing_speed
        odrvYZ.axis1.controller.input_vel = -homing_speed

        if (vel_yz0 > moving_tolerance and vel_yz1 > moving_tolerance):
            moving = True

        if (moving and (vel_yz0 + vel_yz1)/2 < stopped_tolerance):
            odrvYZ.axis0.controller.input_vel = 0
            odrvYZ.axis1.controller.input_vel = 0

            odrvYZ.axis0.encoder.set_linear_count(0)
            odrvYZ.axis1.encoder.set_linear_count(0)

            print("Y homed")
            y_homed = True
            moving = False
            break

# Move all axes away from home for 2s to ensure that homing is possible
print("Moving back X before homing")
odrvXA.axis0.controller.input_vel = -homing_speed

sleep(1)

odrvXA.axis0.controller.input_vel = 0
odrvXA.axis1.controller.input_vel = 0

sleep(2)

print("Homing X")

# Homing YZ Loop
while(True):

    # Constantly update velocity estimates
    vel_x = abs(odrvXA.axis0.encoder.vel_estimate)
    vel_a = abs(odrvXA.axis1.encoder.vel_estimate)

    if (x_homed == False):
        odrvXA.axis0.controller.input_vel = homing_speed

        if (vel_x > moving_tolerance):
            moving = True

        if (moving and vel_x < stopped_tolerance):
            odrvXA.axis0.controller.input_vel = 0

            odrvXA.axis0.encoder.set_linear_count(0)
            
            x_homed = True
            moving = False
            break

# Set odrives back to position control
odrvXA.axis0.controller.input_pos = 0
odrvXA.axis1.controller.input_pos = 0
odrvYZ.axis0.controller.input_pos = 0
odrvYZ.axis1.controller.input_pos = 0

odrvXA.axis0.controller.config.control_mode = 3
odrvXA.axis1.controller.config.control_mode = 3
odrvYZ.axis0.controller.config.control_mode = 3
odrvYZ.axis1.controller.config.control_mode = 3

odrvXA.axis0.requested_state = AXIS_STATE_IDLE
odrvXA.axis1.requested_state = AXIS_STATE_IDLE
odrvYZ.axis0.requested_state = AXIS_STATE_IDLE
odrvYZ.axis1.requested_state = AXIS_STATE_IDLE

print("Homing finished")

