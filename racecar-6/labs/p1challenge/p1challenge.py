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
from typing import Callable, NamedTuple, Optional, Tuple

import numpy as np
import racecar_core
import racecar_utils as rc_utils
import numpy as np
from nptyping import NDArray
from pid import PIDController

import cv2

########################################################################################
# Data
########################################################################################


class State(Enum):
    """
    A state the robot can be in during the cone slalom challenge.
    """

    LINE_FOLLOWING = 0
    APPROACH = 1 
    SWERVE_LEFT = 2
    SWERVE_RIGHT = 3
    PARKED = 4


class Color(Enum):
    """
    Represents a range of colors that the robot can contour,
    with each variable containing a min and max HSV.
    """

    # Sim colors
    BLUE =  ((91-20, 106-45, 206-30), (91+20, 255, 255))

    # Line colors
    YELLOW = ((30-20, 50, 150), (30+20, 255, 255))
    #BLUE = ((90, 110, 110), (120, 255, 255))

    # BLUE = ((91-20, 106-45, 206-30), (91+20, 255, 255))
    GREEN = ((56-30, 66-10, 179-60), (61+30, 100+30, 173+40))

    # Cone colors
    ORANGE = ((30-20, 50, 150), (30+20, 255, 255))
    PURPLE = ((0, 0, 0), (0, 0, 0))

    # Both
    RED = ((0, 0, 0), (0, 0, 0))


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
LINE_COLOR_PRIORITY = (Color.GREEN, Color.BLUE, Color.YELLOW)

FOLLOWING_SPEED = 0.17 if IS_REAL else 1

current_state: State = State.LINE_FOLLOWING
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

# contour_center = None  # The (pixel row, pixel column) of contour
# contour_area = 0  # The area of contour
screen_width = 0  # the width of the screen, in px, because it changes between real and sim

# velocity: NDArray[3, np.float32] = np.ndarray((0, 0, 0))

controller = PIDController(
    k_p=0.075 if IS_REAL else 8.0,
    k_i=0,
    k_d=0.065 if IS_REAL else 0.1,
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

global queue 
queue = []

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
        
            #rc_utils.draw_contour(image, contour.contour)
            #rc.display.show_color_image(image)
        else:
            angle = 1 if angle > 0 else -1
        # print(angle)

        speed = FOLLOWING_SPEED

        orange_cone = get_contour(image, (Color.ORANGE, ), crop_top_two_thirds)
 
        if orange_cone is not None:
            current_state = State.APPROACH

    if current_state == State.APPROACH:
        depth_image = rc.camera.get_depth_image()
        if depth_image is not None:
            
            top_of_frame = crop_top_two_thirds(image)
            
            top_of_frame_orange = cv2.inRange(cv2.cvtColor(top_of_frame, cv2.COLOR_BGR2HSV),
                 Color.ORANGE.value[0],Color.ORANGE.value[1])
            top_of_frame_purple = cv2.inRange(cv2.cvtColor(top_of_frame, cv2.COLOR_BGR2HSV),
                 Color.PURPLE.value[0],Color.PURPLE.value[1])
            
            top_of_frame_color = cv2.bitwise_or(top_of_frame_orange,top_of_frame_purple)
            top_of_frame_depth = cv2.inRange(crop_top_two_thirds(depth_image),
                                            2,
                                            240)
            
            mask = cv2.bitwise_and(top_of_frame_color, top_of_frame_depth)

            #find contours too lazy to find the method
            
            contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
            if len(contours) != 0:
                
                largest_contour_tracking = rc_utils.get_largest_contour(contours)

                if largest_contour_tracking is not None:
                    rc_utils.draw_contour(image,largest_contour_tracking)
                    rc.display.show_color_image(image)
                    

                    
                    angular_offset = rc_utils.remap_range(
                        rc_utils.get_contour_center(largest_contour_tracking)[1],
                        0, 
                        screen_width,
                        -1,
                        1
                    )
            
                    angle = controller.calculate(position=0, setpoint=angular_offset)


                
                orange_pixels_closer_than_60_cm = cv2.inRange(crop_top_two_thirds(depth_image),
                                            2,
                                            90)
                try: 
                    mask = cv2.bitwise_and(top_of_frame_color, orange_pixels_closer_than_60_cm)
                    contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
                    if len(contours) != 0:
                        largest_contour = rc_utils.get_largest_contour(contours)
                        if rc_utils.get_contour_area(largest_contour) > 1000:
                            current_state = State.SWERVE_RIGHT
                            global queue
                            queue.append([0.7,0.4,1])
                            # queue.append([1.8,0.5,-1])
                            # queue.append([0.5,0.2,1])
                        
                except:
                    pass

    if current_state == State.SWERVE_RIGHT:
      

 
        if len(queue) != 0:
            queue[0][0] -= rc.get_delta_time()
            command = queue[0]
            speed = command[1]
            angle = command[2]
            if command[0] <= 0:
                queue.pop(0)

        else:
        
            angle = -1
            depth_image = rc.camera.get_depth_image()
            if depth_image is not None:
                purple = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV),
                 Color.PURPLE.value[0],Color.PURPLE.value[1])
                image_thresh_for_closeness = cv2.inRange(depth_image,
                    2,100)
                close_purple = cv2.bitwise_and(purple,image_thresh_for_closeness)
 
                purple_contours =  cv2.findContours(close_purple, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
                if len(purple) != 0:
                    if rc_utils.get_largest_contour(purple_contours) is not None:
                        
                        if rc_utils.get_largest_contour(purple_contours) > 200:
                            current_state = State.APPROACH
                else:
                    angle = -1
                
            
 
            


    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################
if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
