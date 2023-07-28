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

sys.path.insert(0, "../../library")
from enum import Enum
from typing import Optional

import cv2
import racecar_core
import racecar_utils as rc_utils
from group_6.control import *
from group_6.vision import *


########################################################################################
# State
########################################################################################

class State(Enum):
    """
    A state the robot can be in during the cone slalom challenge.
    """

    LINE_FOLLOWING = 0
    WALL_FOLLOWING = 1
    CONE_SLALOM = 2
    REVERSE = 3
    PARKED = 4

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

IS_SIMULATION = "-s" in sys.argv
IS_REAL = not IS_SIMULATION

LINE_COLOR_PRIORITY = (Color.BLUE, Color.YELLOW)

FOLLOWING_SPEED = 0.15 if IS_REAL else 0.75

# wall following
OFFSET = 7
RIGHT_WINDOW = (90 - OFFSET, 90)
LEFT_WINDOW = (270, 270 + OFFSET)

# cone slalom
RIGHT_CONE = Color.ORANGE
LEFT_CONE = Color.PURPLE

prev_cone: Optional[Color] = None
target_cone: Color = RIGHT_CONE

current_state: State = State.CONE_SLALOM
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
angle_limiter = RateLimiter(0.08) # A filter that can be used by states to smooth angle control

angle_controller = PIDController(
    k_p=0.12 if IS_REAL else 8.0,
    k_i=0,
    k_d=0.055 if IS_REAL else 0.1,
    min_output=-1,
    max_output=1
)

wall_controller = PIDController(
    k_p=0.0045,
    k_i=0.0,
    k_d=0.0002,
    setpoint=0,
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
    # rc.drive.stop()

    global speed
    global angle
    speed = 0.0  # The current speed of the car
    angle = 0.0  # The current angle of the car's wheels

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    global speed, angle, current_state, target_cone, prev_cone

    # depth = get_closest_depth()
    image = rc.camera.get_color_image()
    print(current_state)

    if current_state == State.LINE_FOLLOWING:
        contour = get_contour(image, LINE_COLOR_PRIORITY, crop_floor)

        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if contour is not None:
            print(f"Color: {contour.color}")
            angular_offset = rc_utils.remap_range(contour.center[1], 0, image.shape[1], -1, 1)
            angle = angle_controller.calculate(position=0, setpoint=angular_offset)

        speed = FOLLOWING_SPEED

    if current_state == State.CONE_SLALOM:

        cone = get_contour(image, (target_cone, ), crop_bottom_two_thirds, min_contour_area=300)
        red = get_contour(image, (Color.RED, ), crop_bottom_two_thirds)

        # if red is not None:
        #     current_state = State.PARKED
        #     return

        print(f"prev cone: {prev_cone}, target cone: {target_cone}")

        if cone is not None:
            prev_cone = cone.color

            color_offset = 1 * (-1 if cone.color == LEFT_CONE else 1)
            print(cone.color)

            cone_offset = min(cone.center[1], 0) if cone.color == LEFT_CONE else max(cone.center[1], 0)
            # cone_offset = cone.center[1]

            angular_offset = rc_utils.remap_range(cone_offset, 0, image.shape[1], -1, 1)
            angle = angle_controller.calculate(position=0, setpoint=angular_offset+color_offset)
            print(f"going to {cone.color}")
        elif prev_cone is not None:
            target_cone = RIGHT_CONE if prev_cone == LEFT_CONE else LEFT_CONE
            # angle = angle_limiter.update(0.15 * (1 if target_cone == RIGHT_CONE else -1))
            angle = 0.15 * (1 if target_cone == RIGHT_CONE else -1)

        speed = FOLLOWING_SPEED

    if current_state == State.WALL_FOLLOWING:
        scan = rc.lidar.get_samples()

        _, left_dist = rc_utils.get_lidar_closest_point(scan, LEFT_WINDOW)
        _, right_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_WINDOW)

        angle = wall_controller.calculate(position=left_dist-right_dist)

        speed = FOLLOWING_SPEED

    if current_state == State.PARKED:
        speed = 0.0
        angle = 0.0

    # Display the image to the screen
    rc.display.show_color_image(image)

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################
if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
