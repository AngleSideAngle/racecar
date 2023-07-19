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
from enum import Enum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# State
########################################################################################

class State(Enum):
    LINE_FOLLOWING = 0
    CONE_SLALOM = 1
    PARKED = 2

class Color(Enum, tuple[tuple[int, int, int], tuple[int, int, int]]):

    # Line colors
    YELLOW = ((0, 0, 0), (0, 0, 0))
    BLUE = ((91-20, 106-45, 206-30), (91+20, 110+45, 208+40))
    GREEN = ((56-30, 66-10, 179-60), (61+30, 100+30, 173+40))

    # Cone colors
    ORANGE = ((0, 0, 0), (0, 0, 0))
    PURPLE = ((0, 0, 0), (0, 0, 0))

    # Both
    RED = ((0, 0, 0), (0, 0, 0))


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 60


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
    print(">> Phase 1 Challenge: Cone Slaloming")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Slalom between red and blue cones.  The car should pass to the right of
    # each red cone and the left of each blue cone.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
