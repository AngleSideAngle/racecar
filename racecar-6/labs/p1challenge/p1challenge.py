"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Phase 1 Challenge - Cone Slaloming
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

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
global speed
global angle

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    # rc.drive.stop()

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")
    global controller
    speed = 0.5  # The current speed of the car
    angle = 0.3  # The current angle of the car's wheels
    screen_width = 0 # the width of the screen, in px, because it changes between real and sim
    contour_center = None  # The (pixel row, pixel column) of contour
    contour_area = 0  # The area of contour
    screen_width = 0 # the width of the screen, in px, because it changes between real and sim
    controller = PIDController(
        k_p=5,
        k_i=0,
        k_d=0.1,
        min_output=-1,
        max_output=1
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Initialize variables
    speed = 1
    angle = 1

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)
    # TODO: Slalom between red and blue cones.  The car should pass to the right o
    # each red cone and the left of each blue cone.
    if cur_state == State.SEARCH:
        #setting speed and angle to wander
        speed = 0.4
        angle = 0
        if contour_center:
            cur_state = State.AVOID



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
