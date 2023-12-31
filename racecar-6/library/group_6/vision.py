"""
Group 6's vision library, with helpful utils in using the intel realsense camera
"""

import sys

sys.path.insert(0, "../../library")
from enum import Enum
from typing import Callable, NamedTuple, Optional, Tuple

import numpy as np
import racecar_utils as rc_utils
from nptyping import NDArray

DEFAULT_MIN_CONTOUR_AREA = 500

Color = Tuple[Tuple[int, int, int], Tuple[int, int, int]]
"""
Alias for a range of colors that the robot can contour,
with each variable containing a min and max HSV.
"""

class ContourData(NamedTuple):
    """
    Represents important data about a contour.
    """
    contour : NDArray
    color: Color
    center: Tuple[float, float]
    area: float
    bounds: Tuple[float, float]

    def __repr__(self) -> str:
        return f"ContourData(color={self.color},\ncenter={self.center},\narea={self.area},\nbounds={self.bounds})"

def get_contour(
    image: NDArray,
    color_priority: Tuple[Color, ...],
    cropper: Callable[[NDArray], NDArray] = lambda x: x,
    min_contour_area: int = DEFAULT_MIN_CONTOUR_AREA
) -> Optional[ContourData]:
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """

    if image is None:
        return None

    # Crop the image to the floor directly in front of the car
    image = cropper(image)

    # The contour
    contour = None
    color = None

    for col in color_priority:
        contours = rc_utils.find_contours(image, col[0], col[1])
        contour = rc_utils.get_largest_contour(contours, min_contour_area)
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

        return ContourData(contour, color, contour_center, contour_area, image.shape)  # type: ignore

    return None


def get_closest_depth(depth_image: NDArray) -> Optional[float]:
    """
    Finds the closest depth value
    """

    if depth_image is None:
        return None

    # Crop the image
    depth_image = crop_top_two_thirds(depth_image)

    # Find closest pixel
    closest_pixel = rc_utils.get_closest_pixel(depth_image)

    return depth_image[closest_pixel[0], closest_pixel[1]]

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

def crop_bottom_two_thirds(image: NDArray) -> NDArray:
    """
    Returns the bottom 2/3 of the inputted NDArray
    """
    return rc_utils.crop(image, (image.shape[0] // 3, 0), (image.shape[0], image.shape[1]))

def crop_left(image: NDArray) -> NDArray:
    """
    Return left half of image
    """
    return rc_utils.crop(image, (0, 0), (image.shape[0], image.shape[1] // 2))

def crop_right(image: NDArray) -> NDArray:
    """
    Return right half of image
    """
    return rc_utils.crop(image, (0, image.shape[1] // 2), (image.shape[0], image.shape[1]))

def crop_bottom_3_4ths(image: NDArray) -> NDArray:
    """
    Returns the bottom 2/3 of the inputted NDArray
    """
    return rc_utils.crop(image, (image.shape[0] // 4, 0), (image.shape[0], image.shape[1]))