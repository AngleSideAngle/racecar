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
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

import racecar_core
import racecar_utils as rc_utils
from nptyping import NDArray
from pid import PIDController

########################################################################################
# Data
########################################################################################


class State(Enum):
    """
    A state the robot can be in during the cone slalom challenge.
    """

    LINE_FOLLOWING = 0
    CONE_SLALOM = 1
    PARKED = 2


class Color(Enum):
    """
    Represents a range of colors that the robot can contour,
    with each variable containing a min and max HSV.
    """

    # Line colors
    YELLOW = ((24-10, 102-45, 187-30), (25+20, 113+45, 197+40))
    # BLUE = ((91-20, 106-45, 206-30), (91+20, 110+45, 208+40))
    BLUE = ((90, 50, 50), (120, 255, 255))
    GREEN = ((56-30, 66-10, 179-60), (61+30, 100+30, 173+40))

    # Cone colors
    ORANGE = ((0, 0, 0), (0, 0, 0))
    PURPLE = ((0, 0, 0), (0, 0, 0))

    # Both
    RED = ((0, 0, 0), (0, 0, 0))


@dataclass
class ContourData:
    """
    Represents important data about a contour.
    """

    color: Color
    center: tuple[float, float]
    area: float

########################################################################################
# Global variables
########################################################################################


rc = racecar_core.create_racecar()

# Add any global variables here

# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 500
LINE_COLOR_PRIORITY = (Color.BLUE, Color.GREEN, Color.YELLOW)

current_state: State = State.LINE_FOLLOWING
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

# contour_center = None  # The (pixel row, pixel column) of contour
# contour_area = 0  # The area of contour
screen_width = 0  # the width of the screen, in px, because it changes between real and sim
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
    color_priority: tuple[Color, ...],
    cropper: Callable[[NDArray], NDArray]
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
            print("drawn circle")
            rc_utils.draw_circle(image, contour_center)

        return ContourData(color, contour_center, contour_area)  # type: ignore

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

    global speed
    global angle
    global current_state

    depth = get_closest_depth()
    image = rc.camera.get_color_image()

    if current_state == State.LINE_FOLLOWING:
        contour = get_contour(image, LINE_COLOR_PRIORITY, crop_floor)

        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if contour is not None:
            angular_offset = rc_utils.remap_range(
                contour.center[1], 0, screen_width, -1, 1)
            angle = controller.calculate(position=0, setpoint=angular_offset)
            # print(f"angular offset: {angular_offset}")
            # print(controller)
        else:
            angle = 1 if angle > 0 else -1
        # print(angle)

        speed = 0.325

        orange_cone = get_contour(image, (Color.ORANGE, ), crop_top_two_thirds)

        if orange_cone is not None:
            current_state = State.CONE_SLALOM

    # Display contour onto the image to the screen
    rc.display.show_color_image(image)

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################
if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
