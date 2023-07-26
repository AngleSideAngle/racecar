"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from pid import PIDController
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
# angle = 0
FRONT_WINDOW = (-10, 10)
REAR_WINDOW = (170, 190)
RIGHT_WINDOW = (75, 105)
THRESHOLD = 60
right_dis = 0
controller = PIDController(
    k_p=0.015,
    k_i=0,
    k_d=0,
    setpoint=THRESHOLD,
    min_output=-1,
    max_output=1
)
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")



def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Follow the wall to the right of the car without hitting anything.
    scan = rc.lidar.get_samples()
    _, right_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_WINDOW)
    # right_dist /= 100 # convert to meters
    angle = -controller.calculate(position=right_dist)
    speed = 0.17

    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    _, back_dist = rc_utils.get_lidar_closest_point(scan, REAR_WINDOW)

    # Use the triggers to control the car's speed
    # if forward_dist > THRESHOLD:
    #     speed = 0.2
    # else:
    #     speed = 0

    # if back_dist > THRESHOLD:
    #     speed = 0.2
    # else:
    #     speed = 0
    
    # print(f"forward dist: {forward_dist}")
    # print(f"back dist: {back_dist}")
    # print(f" total: {speed}")
    print(f"right-dist: {right_dist} angle: {angle}")

    rc.drive.set_speed_angle(speed, angle)



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
