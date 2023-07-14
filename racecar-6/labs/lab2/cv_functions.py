
# %%
import cv2 as cv
import numpy as np
from nptyping import NDArray
# %%

import matplotlib.pyplot as plt
import ipywidgets as widgets

from typing import Any, Tuple, List, Optional

def show_image(image: NDArray) -> None:
    """
    Displays a color image in the Jupyter Notebook.
    """
    plt.imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))
    plt.show()


def draw_contour(
    image: NDArray,
    contour: NDArray,
    color: Tuple[int, int, int] = (0, 255, 0)
) -> None:
    """
    Draws a contour on the provided image.

    Args:
        image: The image on which to draw the contour.
        contour: The contour to draw on the image.
        color: The color to draw the contour in BGR format.
    """
    cv.drawContours(image, [contour], 0, color, 3)


def draw_circle(
    color_image: NDArray[(Any, Any, 3), np.uint8],
    center: Tuple[int, int],
    color: Tuple[int, int, int] = (0, 255, 255),
    radius: int = 6,
) -> None:
    """
    Draws a circle on the provided image.

    Args:
        color_image: The color image on which to draw the contour.
        center: The pixel (row, column) of the center of the image.
        color: The color to draw the circle in BGR format.
        radius: The radius of the circle in pixels.
    """
    # cv.circle expects the center in (column, row) format
    cv.circle(color_image, (center[1], center[0]), radius, color, -1)


def show_color_bgr(blue: int, green: int, red: int) -> None:
    """
    Displays a color specified in the BGR format.
    """
    rectangle = plt.Rectangle((0,0), 50, 50, fc=(red/255, green/255, blue/255))
    plt.gca().add_patch(rectangle)
    plt.show()


def show_color_hsv(hue: int, saturation: int, value: int) -> None:
    """
    Displays a color specified in the HSV format
    """
    # Convert from hsv to bgr
    hsv = np.array([[[hue, saturation, value]]], np.uint8)
    bgr = cv.cvtColor(hsv, cv.COLOR_HSV2BGR)

    show_color_bgr(bgr[0][0][0], bgr[0][0][1], bgr[0][0][2])


def get_mask(
    image: NDArray[(Any, Any, 3), np.uint8],
    hsv_lower: Tuple[int, int, int],
    hsv_upper: Tuple[int, int, int]
) -> NDArray[Any, Any]:
    """
    Returns a mask containing all of the areas of image which were between hsv_lower and hsv_upper.

    Args:
        image: The image (stored in BGR) from which to create a mask.
        hsv_lower: The lower bound of HSV values to include in the mask.
        hsv_upper: The upper bound of HSV values to include in the mask.
    """
    # Convert hsv_lower and hsv_upper to numpy arrays so they can be used by OpenCV
    hsv_lower = np.array(hsv_lower)
    hsv_upper = np.array(hsv_upper)

    # TODO: Use the cv.cvtColor function to switch our BGR colors to HSV colors
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # TODO: Use the cv.inRange function to highlight areas in the correct range
    mask = cv.inRange(hsv,hsv_lower,hsv_upper)
    
    return mask


# %%
def find_contours(mask: NDArray) -> List[NDArray]:
    """
    Returns a list of contours around all objects in a mask.
    """
    return cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]

# %% [markdown]
# `find_contours` will return a list containing multiple contours if there are multiple distinct objects which fall between `hsv_lower` and `hsv_upper`.  This might occur if there are multiple cones in the image or if there are other objects that have a similar color to the cone(s).
# 
# Let's write a helper function to identify the largest contour, which we will assume is the closest cone.  This helper function should also ignore contours below a minimum size (such as `30 pixels`), since anything below this size is likely too small to be a cone.
# 
# **<font style="color:red">Finish writing `get_largest_contour` so that it returns the largest contour larger than `min_area`, or `None` if no such contour exists.</font>**  You will likely wish to use the OpenCV [`contourArea`](https://docs.opencv.org/4.2.0/d3/dc0/group__imgproc__shape.html#ga2c759ed9f497d4a618048a2f56dc97f1) function to find the number of pixels in a contour.

# %%
def get_largest_contour(contours: List[NDArray], min_area: int = 30) -> Optional[NDArray]:
    """
    Finds the largest contour with size greater than min_area.

    Args:
        contours: A list of contours found in an image.
        min_area: The smallest contour to consider (in number of pixels)

    Returns:
        The largest contour from the list, or None if no contour was larger than min_area.
    """
    if len(contours) == 0:
        # TODO: What should we return if the list of contours is empty?
        return None

    # TODO: Return the largest contour, but return None if no contour is larger than min_area
    index = 0
    biggest = contours[0]
    while len(contours)>index:
        if contours[index].size>biggest.size:
            biggest = contours[index]
        index = index +1
    if biggest.size<min_area:
        return None
    return biggest

# %% [markdown]
# <a id="ContourCenter"></a>
# ## 6. Contour Center
# 
# One advantage of contours is that we can use them to easily calculate the center of an object.  Specifically, we will use the contour's [_Moments_](https://en.wikipedia.org/wiki/Image_moment), which are weighted averages of the pixels in the contour.  We can calculate the moment $M_{ij}$ with the following formula:
# 
# ```
# def moment(i, j):
#     sum = 0
#     for pixel in contour:
#         sum += pixel.x_position ** i + pixel.y_position ** j
#     return sum
# ```
# 
# To calculate contour center, we will use the following moments:
# 
# * $M_{00}$: The number of pixels in the contour.
# * $M_{10}$: The sum of how far to the right each pixel in the contour is.
# * $M_{01}$: The sum of how far down each pixel in the contour is.
# 
# Using the [center of mass equation](https://en.wikipedia.org/wiki/Center_of_mass), $\frac{M['m10']}{M['m00']}$ gives us the average horizontal position (column) of the contour, and $\frac{M['m01']}{M['m00']}$ gives us the average vertical position (row).

# %%
def get_contour_center(contour: NDArray) -> Optional[Tuple[int, int]]:
    """
    Finds the center of a contour from an image.

    Args:
        contour: The contour of which to find the center.

    Returns:
        The (row, column) of the pixel at the center of the contour, or None if the contour is empty.
    """
    # Ask OpenCV to calculate the contour's moments
    M = cv.moments(contour)

    # Check that the contour is not empty
    if M["m00"] <= 0:
        return None

    # Compute the center of mass of the contour
    center_row = round(M["m01"] / M["m00"])
    center_column = round(M["m10"] / M["m00"])

    return (center_row, center_column)


# In this se
# %%
def clamp(value: float, vmin: float, vmax: float) -> float:
    """
    Clamps a value between a minimum and maximum value.

    Args:
        value: The input to clamp.
        vmin: The minimum allowed value.
        vmax: The maximum allowed value.

    Returns:
        The value saturated between vmin and vmax.
    """
    
    if value > vmax:
        value = vmax
    elif value < vmin:
        value = vmin
        
    return value
    # TODO: Make sure that value is between min and max


# %% [markdown]
# Let's test out our `clamp` function.

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
    percent = val/diff
    new_val = percent * (new_max-new_min)
    new_val += new_min
    
    return new_val
    # TODO: remap val to the new range

