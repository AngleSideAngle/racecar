"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2B - Color Image Cone Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
from pid import PIDController
from enum import IntEnum
########################################################################################
# Global variables
########################################################################################


class State(IntEnum):
    SEARCH = 0
    APPROACH = 1
    STOP = 2


rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# The HSV range for the color orange, stored as (hsv_min, hsv_max)
ORANGE = ((10, 100, 100), (20, 255, 255))

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
cur_state = State.SEARCH #the robot state

########################################################################################
# Functions
########################################################################################


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
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        # Set global screen width
        screen_width = image.shape[1]

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
    global speed
    global angle
    global cur_state 
    

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message

    print(">> Lab 2B - Color Image Cone Parking")
    global controller
    speed = 0.0  # The current speed of the car
    angle = 0.0  # The current angle of the car's wheels
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

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global cur_state

    update_contour()

    # Search for contours in the current color image
   # update_contour()

    #global contour_center
    

    #angular_offset = remap_range(contour_center[1], 0, screen_width, -1, 1)
    #angle = controller.calculate(position=0, setpoint=angular_offset)


     # Use the triggers to control the car's speed
    #forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    #backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    #speed = 0.5 * (forwardSpeed - backSpeed)
    

    rc.drive.set_speed_angle(speed, angle)

    # Here we are initializing the starting state to be search
    cur_state: State = State.SEARCH
    
    if cur_state == State.SEARCH:
        #setting speed and angle to wander
        speed=0.4
        angle=0
        if contour_center:
            cur_state = State.APPROACH

    if cur_state == State.APPROACH:
        angular_offset = remap_range(contour_center[1], 0, screen_width, -1, 1)
        angle = controller.calculate(position=0, setpoint=angular_offset)
        speed = 0.4
        if next_to_cone:
        
        cur_state = State.STOP
            if not cone_identified: 
                cur_state = State.SEARCH
    if cur_state == State.STOP:
        speed = 0
        angle = 0
    







        

    

    # TODO: Park the car 30 cm away from the closest orange cone

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)
    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x position
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
