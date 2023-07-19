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
from typing import *
from nptyping import NDArray
from collections import namedtuple

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

class ContourData(NamedTuple):
    color: Color
    center: tuple[float, float]
    area: float

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

# contour_center = None  # The (pixel row, pixel column) of contour
# contour_area = 0  # The area of contour
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

def get_contour() -> Optional[ContourData]:
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """

    global screen_width

    image = rc.camera.get_color_image()

    if image is None:
        return None

    # Crop the image to the floor directly in front of the car
    image = rc_utils.crop(image, (image.shape[0] // 2, 0), (image.shape[0], image.shape[1]))

    # Set global screen width
    screen_width = image.shape[1]

    # The contour
    contour = None
    color = None

    for c in COLOR_PRIORITY:
        contours = rc_utils.find_contours(image, c.value[0], c.value[1])
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        color = c
        if contour is not None:
            break

    # Draw contour onto the image
    if contour is not None:
        rc_utils.draw_contour(image, contour)
        rc_utils.draw_circle(image, contour_center)

    # Display the image to the screen
    rc.display.show_color_image(image)

    # Calculate and return contour information
    if contour is not None:
        contour_center = rc_utils.get_contour_center(contour)
        contour_area = rc_utils.get_contour_area(contour)
        return ContourData(color, contour_center, contour_area) # type: ignore

    return None

def get_closest_depth() -> Optional[float]:
    """
    Finds the closest depth value
    """

    depth_image = rc.camera.get_depth_image()

    if depth_image is None:
        return None

    # Crop the image
    top_left_inclusive = (0, 0)
    bottom_right_exclusive = (depth_image.shape[0] * 2 // 3, depth_image.shape[1])

    depth_image = rc_utils.crop(depth_image, top_left_inclusive, bottom_right_exclusive)

    # Find closest pixel
    closest_pixel = rc_utils.get_closest_pixel(depth_image)

    # Telemetry
    rc.display.show_depth_image(depth_image, points=[closest_pixel])

    return depth_image[closest_pixel[0], closest_pixel[1]]

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
    
    contour = get_contour()
    depth = get_closest_depth()

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
