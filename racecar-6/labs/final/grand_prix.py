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
from typing import Any, List, Tuple
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from group_6.control import *
from group_6.vision import *
from group_6.utils import *
from cone_slalom import ConeSlalom
from line_following import LineFollowing, LaneFollowing
from wall_following import RightWallFollowing, CenterWallFollowing

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
speed_limiter = RateLimiter(0.1)

# Add any global variables here

current_state: State
current_data: RobotData # read only data for passing to states
visible_tags: List[rc_utils.ARMarker] = []


########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global current_state

    # Have the car begin at a stop
    rc.drive.stop()

    rc.set_update_slow_time(0.08)

    # Print start message
    print(">> Final Challenge - Grand Prix")

    current_state = ConeSlalom()


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    global current_state, current_data

    current_data = RobotData(
        image=rc.camera.get_color_image(),
        visible_tags=visible_tags,
        lidar_scan=rc.lidar.get_samples()
    )

    speed, angle = current_state(current_data)
    speed = speed_limiter.update(speed)

    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

def update_slow() -> None:
    """
    Slow update function, updates ar markers and prints diagnostic data
    """

    global visible_tags

    image = rc.camera.get_color_image()
    visible_tags = rc_utils.get_ar_markers(image)

    print(f"TIME ELAPSED: {rc.get_delta_time()}", current_state, current_data, sep="\n")

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
