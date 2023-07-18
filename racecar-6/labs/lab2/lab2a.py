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
from nptyping import NDArray

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
MIN_CONTOUR_AREA = 50

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((91-20, 106-45, 206-30), (91+20, 110+45, 208+40))  # The HSV range for the color blue
# BLUE = ((90, 50, 50), (120, 255, 255))
RED = ((160-20, 111-45, 182-40), (179, 157+45, 255))
GREEN = ((56-30, 66-10, 179-60), (61+30, 100+30, 173+40))

color_num_to_str = { 
    0 : "blue",
    1 : "red",
    2 : "green"
}

hsv_num_to_str = { 
    0 : "Hue",
    1 : "Saturation",
    2 : "Value"
}
global color_idx
global HSV_idx
global threshold_incremement

color_idx = 0
HSV_idx = 0
threshold_incremement = 5


color_idx_to_tuple = {
    0 : BLUE,
    1 : RED,
    2 : GREEN
}

# def update_based_on_taps():
#     global color_idx
#     global HSV_idx

#     if rc.controller.was_pressed(rc.controller.Button.A):
#         color_idx = (color_idx + 1) % 3 
#     if rc.controller.was_pressed(rc.controller.Button.B):
#         HSV_idx = (HSV_idx + 1) % 3 
#     if rc.controller.was_pressed(rc.controller.Button.Y):
#         color_idx_to_tuple[color_idx][1][HSV_idx] = max(
#                                 color_idx_to_tuple[color_idx][1][HSV_idx] + threshold_incremement, 255)
#     if rc.controller.was_pressed(rc.controller.Button.X):
#         color_idx_to_tuple[color_idx][1][HSV_idx] = min(
#                                 color_idx_to_tuple[color_idx][1][HSV_idx] - threshold_incremement, 0)
    
#     draw_image(rc.camera.get_color_image(), color_idx_to_tuple[color_idx])

#     print(f"{color_num_to_str[color_idx]} : {color_idx_to_tuple[color_idx]}")
#     print(f"HSV_idx : {HSV_idx}, color_idx : {color_idx}")

# def draw_image(image, tuple):
#     contours = rc_utils.find_contours(image, tuple[0], tuple[1])

#     # Select the largest contour
#     contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

#     if contour is None:
#         rc.display.show_color_image(image)
#     else:
#         contour_center = rc_utils.get_contour_center(contour)
#         contour_area = rc_utils.get_contour_area(contour)

#         # Draw contour onto the image
#         rc_utils.draw_contour(image, contour)
#         rc_utils.draw_circle(image, contour_center)
#         rc.display.show_color_image(image)
# # TODO (challenge 1): add HSV ranges for other colors
PRIORITY = (RED, BLUE, GREEN)


# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
screen_width = 0 # the width of the screen, in px, because it changes between real and sim
controller = PIDController(
    k_p=0.2,
    k_i=0,
    k_d=0.04,
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

def crop_floor(image: NDArray) -> NDArray:
    return rc_utils.crop(image, (image.shape[0] // 2, 0), (image.shape[0], image.shape[1]))

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global screen_width

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = crop_floor(image)
        contour_list = []
        for color in PRIORITY:
            # Find all of the current color's contours
            contours = rc_utils.find_contours(image, color[0], color[1])

            # Set global screen width
            screen_width = image.shape[1]

            # Select the largest contour
            contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
            
            contour_list.append(contour_list)
            if contour is None:
                continue
            else:
                print(f"area: {cv.contourArea(contour)}")
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)
                # Draw contour onto the image
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)
                rc.display.show_color_image(image)
                return
        
        for color in PRIORITY:
            new_contours = rc_utils.find_contours(image, color[0], color[1])

            # Set global screen width
            screen_width = image.shape[1]

            # Select the largest contour
            contour = rc_utils.get_largest_contour(new_contours, MIN_CONTOUR_AREA)
            # print(f"area: {cv.contourArea(contour)}")
            contour_list.append(contour_list)
            if contour is None:
                continue
            else:
                new_contour_center = rc_utils.get_contour_center(contour)
                #contour_area = rc_utils.get_contour_area(contour)
                # Draw contour onto the image
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, new_contour_center)
                rc.display.show_color_image(image)
                
        contour_center = None
        contour_area = 0


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
        angular_offset = remap_range(contour_center[1], 0, screen_width, -1, 1)
        angle = controller.calculate(position=0, setpoint=angular_offset)
        print(f"angular offset: {angular_offset}")
        print(controller)
    else:
        angle = 1 if angle > 0 else -1
    print(angle)

    # Use the triggers to control the car's speed
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = 0.3 * (forwardSpeed - backSpeed)
    # speed = 1 # testaing

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)
    # update_based_on_taps()
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
