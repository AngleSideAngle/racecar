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
from pid import PIDController

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
    YELLOW = ((24-10, 102-45, 187-30), (25+20, 113+45, 197+40))
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
COLOR_PRIORITY = (Color.GREEN, )

speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
screen_width = 0 # the width of the screen, in px, because it changes between real and sim
controller = PIDController(
    k_p=0.19,
    k_i=0,
    k_d=0.055,
    min_output=-1,
    max_output=1
)


########################################################################################
# Functions
########################################################################################


def crop_floor(image: NDArray) -> NDArray:
    return rc_utils.crop(image, (image.shape[0] // 2, 0), (image.shape[0], image.shape[1]))

def get_contour(image: NDArray, color) -> Optional[NDArray]:
    contours = rc_utils.find_contours(image, color[0], color[1])
    return rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global screen_width
    global color_index

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:

        # Crop the image to the floor directly in front of the car
        image = crop_floor(image)

        # Set global screen width
        screen_width = image.shape[1]

        # The contour
        contour = None

        for color in COLOR_PRIORITY:
            contour = get_contour(image, color) # ); my precious := operator
            if contour is not None:
                break

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            
        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    # rc.drive.stop()

    global speed
    global angle
    speed = 0.0  # The current speed of the car
    angle = 0.0  # The current angle of the car's wheels

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")


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
