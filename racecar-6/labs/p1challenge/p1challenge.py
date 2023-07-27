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
import time

sys.path.insert(0, "../../library")
from enum import Enum
from typing import Callable, NamedTuple, Optional, Tuple

import cv2
import numpy as np
import racecar_core
import racecar_utils as rc_utils
from nptyping import NDArray
from control import PIDController

########################################################################################
# Data
########################################################################################


class State(Enum):
    """
    A state the robot can be in during the cone slalom challenge.
    """

    LINE_FOLLOWING = 0
    CONE_SLALOM = 2
    PARKED = 4

class Color(Enum):
    """
    Represents a range of colors that the robot can contour,
    with each variable containing a min and max HSV.
    """

    # Line colors
    YELLOW = ((30-20, 50, 150), (30+20, 255, 255))
    # BLUE = ((90, 110, 110), (120, 255, 255))
    BLUE = ((91-20, 106-45, 206-30), (91+20, 255, 255))
    GREEN = ((56-30, 66-10, 179-60), (61+30, 100+30, 173+40))

    # Cone colors
    ORANGE = ((0, 209, 220), (8+20, 255, 255))
    PURPLE = ((147-20, 128-45, 120), (160, 255, 255))
    # Both
    RED = ((150-20, 85-45, 190-30), (179, 255, 255))
    #BLUE = ((90, 110, 110), (120, 255, 255))


class ContourData(NamedTuple):
    """
    Represents important data about a contour.
    """
    contour : NDArray
    color: Color
    center: Tuple[float, float]
    area: float

########################################################################################
# Global variables
########################################################################################


rc = racecar_core.create_racecar()

# Add any global variables here

IS_SIMULATION = "-s" in sys.argv
IS_REAL = not IS_SIMULATION

# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 500
LINE_COLOR_PRIORITY = (Color.BLUE, Color.YELLOW)

FOLLOWING_SPEED = 0.14 if IS_REAL else 0.5

# GRAVITY: NDArray = np.array((0.0, -9.81, 0.0))

current_state: State = State.CONE_SLALOM
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

# contour_center = None  # The (pixel row, pixel column) of contour
# contour_area = 0  # The area of contour
screen_width = 0  # the width of the screen, in px, because it changes between real and sim

position = np.array((0.0, 0.0, 0.0))
velocity = np.array((0.0, 0.0, 0.0))
acceleration = np.array((0.0, 0.0, 0.0))

angular_velocity = np.array((0.0, 0.0, 0.0))
angular_position = np.array((0.0, 0.0, 0.0))

angle_controller = PIDController(
    k_p=0.12 if IS_REAL else 8.0,
    k_i=0,
    k_d=0.055 if IS_REAL else 0.1,
    min_output=-1,
    max_output=1
)

accel_controller = PIDController(
    k_p=0.1,
    k_i=0,
    k_d=0,
    max_output=1,
    min_output=-1
)

########################################################################################
# Functions
########################################################################################

def crop_floor(image: NDArray) -> NDArray:
    """
    Returns the bottom half of the inputted NDArray
    """
    return rc_utils.crop(image, (image.shape[0] // 2, 0), (image.shape[0], image.shape[1]))


def crop_top_two_thirds(image: NDArray) -> NDArray:
    """
    Returns the top 2/3 of the inputted NDArray
    """
    return rc_utils.crop(image, (0, 0), (image.shape[0] * 2 // 3, image.shape[1]))


def get_contour(
    image: NDArray,
    color_priority: Tuple[Color, ...],
    cropper: Callable[[NDArray], NDArray] = lambda x: x
) -> Optional[ContourData]:
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """

    global screen_width

    image = rc.camera.get_color_image()

    if image is None:
        return None

    # Crop the image to the floor directly in front of the car
    image = cropper(image)

    # Set global screen width
    screen_width = image.shape[1]

    # The contour
    contour = None
    color = None

    for col in color_priority:
        contours = rc_utils.find_contours(image, col.value[0], col.value[1])
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        color = col
        if contour is not None:
            break

    # Calculate and return contour information
    if contour is not None:
        contour_center = rc_utils.get_contour_center(contour)
        contour_area = rc_utils.get_contour_area(contour)

        rc_utils.draw_contour(image, contour)

        if contour_center:
            rc_utils.draw_circle(image, contour_center)

        return ContourData(contour, color, contour_center, contour_area)  # type: ignore

    return None


def get_closest_depth() -> Optional[float]:
    """
    Finds the closest depth value
    """

    depth_image = rc.camera.get_depth_image()

    if depth_image is None:
        return None

    # Crop the image
    depth_image = crop_top_two_thirds(depth_image)

    # Find closest pixel
    closest_pixel = rc_utils.get_closest_pixel(depth_image)

    # Telemetry
    # rc.display.show_depth_image(depth_image, points=[closest_pixel])

    return depth_image[closest_pixel[0], closest_pixel[1]]

ACCEL_AVG_LEN = 7
moving_avg = [np.array((0, 0, 0)) for _ in range(7)]

def update_odometry():
    """
    Updates simple odometry based on imu
    """

    global moving_avg, position, velocity, acceleration, angular_velocity, angular_position

    moving_avg.pop(0)
    moving_avg.append(rc.physics.get_linear_acceleration())

    angular_velocity = rc.physics.get_angular_velocity()
    angular_position += rc.physics.get_angular_velocity() * rc.get_delta_time()

    acceleration = sum(moving_avg) / len(moving_avg)
    # print(acceleration)
    velocity += acceleration * rc.get_delta_time()
    position += velocity * rc.get_delta_time()
    print(angular_velocity)


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

cone_priority = (Color.ORANGE, Color.RED, Color.PURPLE)
last_cone_color = Color.ORANGE

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    global speed
    global angle
    global current_state

    # depth = get_closest_depth()
    image = rc.camera.get_color_image()
    # update_odometry()
    print(current_state)

    if current_state == State.LINE_FOLLOWING:
        contour = get_contour(image, LINE_COLOR_PRIORITY, crop_floor)

        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if contour is not None:
            print(f"Color: {contour.color}")
            angular_offset = rc_utils.remap_range(contour.center[1], 0, screen_width, -1, 1)
            angle = angle_controller.calculate(position=0, setpoint=angular_offset)

        speed = FOLLOWING_SPEED

    if current_state == State.CONE_SLALOM:
        cone = get_contour(image, cone_priority, crop_top_two_thirds)
        red = get_contour(image, (Color.RED, ), crop_top_two_thirds)

        # if red is not None:
        #     current_state = State.PARKED
        #     return

        if cone is not None:
            offset = cone.area / 5 * (-1 if cone.color == Color.PURPLE else 1)
            print(cone.color)
            print(offset)
            angular_offset = rc_utils.remap_range(cone.center[1] + offset, 0, screen_width, -1, 1)
            angle = angle_controller.calculate(position=0, setpoint=angular_offset)
            print(f"going to {cone.color}")


        speed = FOLLOWING_SPEED


    if current_state == State.PARKED:
        speed = 0.0
        angle = 0.0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################
if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
