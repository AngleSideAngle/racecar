"""
Copyright MIT and Harvey Mudd College
MIT License
Fall 2020

Final Challenge - Grand Prix
"""

########################################################################################
# Imports
########################################################################################

import sys
from typing import List, Tuple
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from group_6.control import *
from group_6.vision import *
from group_6.states import State


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

IS_SIMULATION = "-s" in sys.argv
IS_REAL = not IS_SIMULATION

GENERAL_COLOR_PRIORITY = (Color.BLUE, Color.RED, Color.GREEN, Color.YELLOW)
END_COLOR_PRIORITY = (Color.BLUE, )

FOLLOWING_SPEED = 0.13 if IS_REAL else 0.75

visible_tags: List[rc_utils.ARMarker] = []

def visible_ids():
    return map(lambda tag: tag.get_id(), visible_tags)

class LineFollowing(State):

    controller = PIDController(
        k_p=0.16 if IS_REAL else 8.0,
        k_i=0,
        k_d=0.005 if IS_REAL else 0.1,
        min_output=-1,
        max_output=1
    )

    debouncer = Debouncer(baseline=True, debounce_time=0.3)

    def __init__(self, color_priority) -> None:
        self.color_priority = color_priority

    def execute(self) -> Tuple[float, float]:
        speed = 0
        angle = 0

        image = rc.camera.get_color_image()

        contour = get_contour(image, self.color_priority, crop_floor, min_contour_area=600)

        in_sight = self.debouncer.update(contour is not None)

        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if contour is not None:
            print(f"Color: {contour.color}")
            angular_offset = rc_utils.remap_range(contour.center[1], 0, image.shape[1], -1, 1)
            angle = self.controller.calculate(position=0, setpoint=angular_offset)

        if in_sight:
            speed = FOLLOWING_SPEED
        else:
            speed = -FOLLOWING_SPEED - 0.1

        return (speed, angle)

    def next_state(self) -> State:
        ids = visible_ids()
        if 4 in ids:
            return WallFollowing()
        if 3 in ids:
            return Stopped()
        return self

class WallFollowing(State):

    OFFSET = 7
    RIGHT_WINDOW = (90 - OFFSET, 90)
    LEFT_WINDOW = (270, 270 + OFFSET)

    controller = PIDController(
        k_p=0.004,
        k_i=0.0,
        k_d=0.004,
        setpoint=70,
        min_output=-1,
        max_output=1
    )

    def execute(self) -> Tuple[float, float]:
        scan = rc.lidar.get_samples()

        # _, left_dist = rc_utils.get_lidar_closest_point(scan, LEFT_WINDOW)
        _, right_dist = rc_utils.get_lidar_closest_point(scan, self.RIGHT_WINDOW)

        # angle = wall_controller.calculate(position=left_dist-right_dist)
        angle = -self.controller.calculate(position=right_dist)

        speed = FOLLOWING_SPEED

        return (speed, angle)

    def next_state(self) -> State:
        ids = visible_ids()
        if 1 in ids:
            return ConeSlalom()
        return self

class ConeSlalom(State):

    # cone slalom
    RIGHT_CONE = Color.ORANGE
    LEFT_CONE = Color.PURPLE

    controller = PIDController(
        k_p=0.15 if IS_REAL else 8.0,
        k_i=0,
        k_d=0.005 if IS_REAL else 0.1,
        min_output=-1,
        max_output=1
    )

    debouncer = Debouncer(baseline=True, debounce_time=0.3)

    prev_cone: Optional[Color] = None
    target_cone: Color = RIGHT_CONE

    seen_cones = 0
    CONE_THRESHOLD = 4

    def execute(self) -> Tuple[float, float]:
        angle = 0

        image = rc.camera.get_color_image()
        cone = get_contour(image, (self.target_cone, ), crop_bottom_two_thirds)

        print(f"prev cone: {self.prev_cone}, target cone: {self.target_cone}")

        in_sight = self.debouncer.update(cone is not None)

        if cone is not None:
            self.prev_cone = cone.color

            print(f"cone color: {cone.color}")
            color_offset = 1 * (-1 if cone.color == self.LEFT_CONE else 1)

            cone_offset = min(cone.center[1], 0) if cone.color == self.LEFT_CONE else max(cone.center[1], 0)
            angular_offset = rc_utils.remap_range(cone_offset, 0, image.shape[1], -1, 1)

            angle = self.controller.calculate(position=0, setpoint=angular_offset+color_offset)
            print(f"going to {cone.color}")
        elif self.prev_cone is not None and not in_sight:
            target_cone = self.RIGHT_CONE if self.prev_cone == self.LEFT_CONE else self.LEFT_CONE
            # angle = angle_limiter.update(0.15 * (1 if target_cone == RIGHT_CONE else -1))
            angle = 0.15 * (1 if target_cone == self.RIGHT_CONE else -1)

        return (FOLLOWING_SPEED, angle)

    def next_state(self) -> State:
        if self.seen_cones >= self.CONE_THRESHOLD:
            return LineFollowing(END_COLOR_PRIORITY)
        return self

class LaneFollow(State):

    controller = PIDController(
        k_p=0.16 if IS_REAL else 8.0,
        k_i=0,
        k_d=0.005 if IS_REAL else 0.1,
        setpoint=0,
        min_output=-1,
        max_output=1
    )

    def __init__(self, color: Color) -> None:
        self.color = color

    def execute(self) -> Tuple[float, float]:
        angle = 0
        speed = FOLLOWING_SPEED

        image = crop_floor(rc.camera.get_color_image())

        left = get_contour(image, (self.color, ), crop_left)
        right = get_contour(image, (self.color, ), crop_right)

        print(f"seen left: {left.center if left is not None else 'NONE'}, seen right: {right.center if right is not None else 'NONE'}")

        if left is None or right is None:
            return (0, 0)

        left_dist = left.bounds[1] - left.center[1]
        offset = left_dist-right.center[1]
        position = rc_utils.remap_range(offset, -image.shape[1] / 2, image.shape[1] / 2, -1, 1)

        angle = self.controller.calculate(position=position)

        return (speed, angle)
    
    def next_state(self):
        return self

class Stopped(State):

    def execute(self) -> Tuple[float, float]:
        return (0, 0)

    def next_state(self) -> State:
        return self

current_state = LaneFollow(color=Color.PURPLE) # LineFollowing(GENERAL_COLOR_PRIORITY)


########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    rc.set_update_slow_time(0.08)

    # Print start message
    print(">> Final Challenge - Grand Prix")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    global current_state

    speed, angle = current_state.execute()
    current_state = current_state.next_state()

    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

def update_ar_markers():
    global visible_tags

    image = rc.camera.get_color_image()
    visible_tags = rc_utils.get_ar_markers(image) # check colors too
    for tag in visible_tags:
        print(f"SEEN TAG: {tag.get_id()}")

if __name__ == "__main__":
    rc.set_start_update(start, update, update_ar_markers)
    rc.go()
