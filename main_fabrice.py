#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function 
import time
import math
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1 
from pymavlink.dialects.v20 import ardupilotmega as mavlink2 

# DO NOT USE THIS CODE DIRECTLY ON A REAL ROBOT WITHOUT CHECKING WHAT IT MIGHT DO!

# See http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html and https://mavlink.io/en/messages/common.html

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)    
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5    
    return [w, x, y, z]

#autopilot = mavutil.mavlink_connection("COM12", 115200)
autopilot = mavutil.mavlink_connection('tcp:192.168.2.2:5777', source_system=255)
#autopilot = mavutil.mavlink_connection('udp:0.0.0.0:14550')
autopilot.wait_heartbeat()
n = 4; dt = 0.1 # To send repeatedly the messages in case of unreliable network
# Arm
print("Arming")
for i in range(0,n):
    autopilot.mav.command_long_send(autopilot.target_system, autopilot.target_component, mavlink1.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(dt)
time.sleep(3)
# Guided (15), Guided_NoGPS mode not yet supported for ArduRover...?
# See enum control_mode_t in https://github.com/ArduPilot/ardupilot/blob/autopilot/Rover/defines.h, https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.h
for i in range(0,n):
    autopilot.mav.set_mode_send(autopilot.target_system, mavlink1.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
    time.sleep(dt)
time.sleep(1)

# Move
print("Forward")
t = 10
for i in range(0,int(t/dt)):
    autopilot.mav.set_attitude_target_send(0, autopilot.target_system, autopilot.target_component, 0b00000111, to_quaternion(0,0,0), 0, 0, 0, thrust=0.1)
    time.sleep(dt)
print("Turn")
t = 10
for i in range(0,int(t/dt)):
    autopilot.mav.set_attitude_target_send(0, autopilot.target_system, autopilot.target_component, 0b00000111, to_quaternion(0,0,-1.57), 0, 0, 0, thrust=0.1)
    time.sleep(dt)
print("Stop")
for i in range(0,n):
    autopilot.mav.set_attitude_target_send(0, autopilot.target_system, autopilot.target_component, 0b00000111, to_quaternion(0,0,0), 0, 0, 0, thrust=0)
    time.sleep(dt)
time.sleep(2)

for i in range(0,n):
    autopilot.mav.set_mode_send(autopilot.target_system, mavlink1.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 1)
    time.sleep(dt)
time.sleep(1)
autopilot.close()
