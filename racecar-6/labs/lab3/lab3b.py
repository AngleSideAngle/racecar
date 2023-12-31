"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3B - Depth Camera Cone Parking
"""

########################################################################################
# Imports
########################################################################################

from enum import Enum
import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
from racecar_core import rc
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

class State(Enum):
    SEARCHING = 0
    FOLLOWING = 1
    PARKED = 2

rc = racecar_core.create_racecar()

# Add any global variables here

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
    print(">> Lab 3B - Depth Camera Cone Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Park the car 30 cm away from the closest orange cone.
    # Use both color and depth information to handle cones of multiple sizes.
    # You may wish to copy some of your code from lab2b.py
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
