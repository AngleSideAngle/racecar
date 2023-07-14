"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2A - Color Image Line Following
"""

########################################################################################
# Imports
########################################################################################

from enum import Enum
import sys
import cv2 as cv
import numpy as np

import cv_functions

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
from pid import PIDController

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((rc.camera.get_height() // 4, 0), (rc.camera.get_height() // 2, rc.camera.get_width() // 2))

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (120, 255, 255))  # The HSV range for the color blue
RED = ((), ())
GREEN = ((), ())

# TODO (challenge 1): add HSV ranges for other colors
PRIORITY = (GREEN, BLUE, RED)



        


# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
controller = PIDController(
    k_p=9,
    k_i=0,
    k_d=0.1,
    min_output=-1,
    max_output=1
)

########################################################################################
# Functions
########################################################################################

# from the lab2 notebook
def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
) -> float:
    """
    Remaps a value from one range to another range.

    Args:
        val: A number form the old range to be rescaled.
        old_min: The inclusive 'lower' bound of the old range.
        old_max: The inclusive 'upper' bound of the old range.
        new_min: The inclusive 'lower' bound of the new range.
        new_max: The inclusive 'upper' bound of the new range.

    Note:
        min need not be less than max; flipping the direction will cause the sign of
        the mapping to flip.  val does not have to be between old_min and old_max.
    """

    diff = old_max - old_min
    percent = val / diff
    new_val = percent * (new_max - new_min)
    new_val += new_min

    return new_val

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()
    
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        for color in PRIORITY:
            hsv_lower = color[0]
            hsv_upper = color[1]
            # Find all of the current color's contours
            contours = rc_utils.find_contours(image, color[0], color[1])

            # Select the largest contour
            contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

            if contour is None:
                continue
            else:
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)
                rc.display.show_color_image(image)
                return 
        contour_center = None
        contour_area = 0


            
   
        # # TODO (challenge 1): Search for multiple tape colors with a priority order
        # # (currently we only search for blue)

        # # Crop the image to the floor directly in front of the car
        # image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # # Find all of the blue contours
        # contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])

        # # Select the largest contour
        # contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        # if contour is not None:
        #     # Calculate contour information
        #     contour_center = rc_utils.get_contour_center(contour)
        #     contour_area = rc_utils.get_contour_area(contour)

        #     # Draw contour onto the image
        #     rc_utils.draw_contour(image, contour)
        #     rc_utils.draw_circle(image, contour_center)

        # else:
        #     contour_center = None
        #     contour_area = 0

        # # Display the image to the screen
        # rc.display.show_color_image(image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    A button = print current speed and angle\n"
        "    B button = print contour center and area"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle

    # Search for contours in the current color image
    update_contour()

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if contour_center is not None:
        angular_offset = remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
        angle = controller.calculate(position=0, setpoint=angular_offset)
        print(f"angular offset: {angular_offset}")
        print(controller)
    else:
        angle = 1 if angle > 0 else -1
    print(angle)

    # Use the triggers to control the car's speed
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = 0.5 * (forwardSpeed - backSpeed)
    # speed = 1 # testing

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
