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
    BRAKING = 4
    PARKED = 5

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

IS_SIMULATION = "-s" in sys.argv
IS_REAL = not IS_SIMULATION

LINE_COLOR_PRIORITY = (Color.BLUE, Color.RED, Color.GREEN, Color.YELLOW)

FOLLOWING_SPEED = 0.14 if IS_REAL else 0.75

# wall following
OFFSET = 7
RIGHT_WINDOW = (90 - OFFSET, 90)
LEFT_WINDOW = (270, 270 + OFFSET)

line_debouncer = Debouncer(baseline=True, debounce_time=0.3)
cone_debouncer = Debouncer(baseline=True, debounce_time=0.3)

# cone slalom
RIGHT_CONE = Color.ORANGE
LEFT_CONE = Color.PURPLE

# reversing
BRAKING_TIME = 0.5
end_breaking = 0

prev_cone: Optional[Color] = None
target_cone: Color = RIGHT_CONE

current_state: State = State.LINE_FOLLOWING
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
angle_limiter = RateLimiter(0.08) # A filter that can be used by states to smooth angle control

angle_controller = PIDController(
    k_p=0.15 if IS_REAL else 8.0,
    k_i=0,
    k_d=0.005 if IS_REAL else 0.1,
    min_output=-1,
    max_output=1
)

wall_controller = PIDController(
    k_p=0.004,
    k_i=0.0,
    k_d=0.001,
    setpoint=70,
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
    rc.set_update_slow_time(0.08)

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")

orange_time = 0
ORANGE_TIMER = 8

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    global speed, angle, current_state, target_cone, prev_cone, end_breaking, orange_time

    # depth = get_closest_depth()
    image = rc.camera.get_color_image()
    print(current_state)

    if current_state == State.LINE_FOLLOWING:
        contour = get_contour(image, LINE_COLOR_PRIORITY, crop_floor, min_contour_area=600)

        in_sight = line_debouncer.update(contour is not None)

        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if contour is not None:
            print(f"Color: {contour.color}")
            angular_offset = rc_utils.remap_range(contour.center[1], 0, image.shape[1], -1, 1)
            angle = angle_controller.calculate(position=0, setpoint=angular_offset)

        if in_sight:
            speed = FOLLOWING_SPEED
        else:
            speed = -FOLLOWING_SPEED - 0.08
            angle = 0

        tags = [tag.get_id() for tag in visible_tags]
        if 4 in tags:
            current_state = State.WALL_FOLLOWING
            # entering wall following
            orange_time = time.perf_counter() + ORANGE_TIMER
        elif 3 in tags:
            end_breaking = time.perf_counter() + BRAKING_TIME
            current_state = State.BRAKING

    if current_state == State.CONE_SLALOM:

        cone = get_contour(image, (target_cone, ), crop_bottom_two_thirds)

        print(f"prev cone: {prev_cone}, target cone: {target_cone}")

        in_sight = cone_debouncer.update(cone is not None)

        if cone is not None:
            prev_cone = cone.color

            print(f"cone color: {cone.color}")
            color_offset = 1 * (-1 if cone.color == LEFT_CONE else 1)

            cone_offset = min(cone.center[1], 0) if cone.color == LEFT_CONE else max(cone.center[1], 0)
            angular_offset = rc_utils.remap_range(cone_offset, 0, image.shape[1], -1, 1)
            
            angle = angle_controller.calculate(position=0, setpoint=angular_offset+color_offset)
            print(f"going to {cone.color}")
        elif prev_cone is not None and not in_sight:
            target_cone = RIGHT_CONE if prev_cone == LEFT_CONE else LEFT_CONE
            # angle = angle_limiter.update(0.15 * (1 if target_cone == RIGHT_CONE else -1))
            angle = 0.15 * (1 if target_cone == RIGHT_CONE else -1)

        speed = FOLLOWING_SPEED

        if time.perf_counter() > orange_time and get_contour(image, (Color.BLUE, ), crop_floor):
            current_state = State.LINE_FOLLOWING

    if current_state == State.WALL_FOLLOWING:
        scan = rc.lidar.get_samples()

        # _, left_dist = rc_utils.get_lidar_closest_point(scan, LEFT_WINDOW)
        _, right_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_WINDOW)

        # angle = wall_controller.calculate(position=left_dist-right_dist)
        angle = -wall_controller.calculate(position=right_dist)

        speed = FOLLOWING_SPEED

        if get_contour(image, (Color.ORANGE, ), crop_bottom_two_thirds, 700):
            current_state = State.CONE_SLALOM

    if current_state == State.BRAKING:
        speed = -0.5
        angle = -0.5

        if time.perf_counter() > end_breaking:
            current_state = State.PARKED

    if current_state == State.PARKED:
        speed = 0.0
        angle = 0.0

    # Display the image to the screen
    rc.display.show_color_image(image)

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

visible_tags = []

def update_ar_markers():
    global visible_tags

    image = rc.camera.get_color_image()
    visible_tags = rc_utils.get_ar_markers(image) # check colors too
    for tag in visible_tags:
        print(f"SEEN TAG: {tag.get_id()}")

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################
if __name__ == "__main__":
    rc.set_start_update(start, update, update_ar_markers)
    rc.go()
