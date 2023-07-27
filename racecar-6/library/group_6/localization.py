"""
WIP: Group 6's library for localization utilities

Carlos Fernández, Vidal Moreno, Belen Curto, J. Andres Vicente,
Clustering and line detection in laser range measurements,
Robotics and Autonomous Systems,
Volume 58, Issue 5,
2010,
Pages 720-726,
ISSN 0921-8890,
https://doi.org/10.1016/j.robot.2009.10.008.
(https://www.sciencedirect.com/science/article/pii/S0921889009001900)
Abstract: This article presents two algorithms that extract information from laser range data. They are designed to work sequentially. The first method (dcc) separates the data into clusters by means of a convolution operation, using a high-pass filter. The second one (reholt) performs line detection in each of the clusters previously discovered. The reliability of the algorithms devised is tested on the experimental data collected both indoors and outdoors. When compared with other methods found in the literature, the ones proposed here prove to achieve higher performance.
Keywords: Feature extraction; Robot mapping
"""

import math

from typing import Tuple

import numpy as np
from nptyping import NDArray

K_1 = np.array((-3, -3, 5, -3, -3))
K_2 = np.array((-1, -2, -3, 5, -3, -2, -1))

def dist(p_1: Tuple[float, float], p_2: Tuple[float, float]):
    """
    Distance between two cartiesian points
    """

    return math.sqrt((p_1[0] - p_2[0]) ** 2 + (p_1[1] - p_2[1]) ** 2)

def to_cartesian(polar: Tuple[float, float]):
    """
    Converts polar coordinates to cartesian
    """

    return (polar[0] * math.cos(polar[1]), polar[0] * math.sin(polar[1]))

def lidar_to_rad(index: int) -> float:
    """
    Converts imu index value (1/2 deg) to radians
    """

    return index / 2.0 * math.pi / 180.0

def dcc(scan_points: NDArray, kernel: NDArray = K_1, deviations: float = 5) -> list[list[tuple[float, float]]]:
    """
    Performs Distance-based Convolution Clustering on a set of scan points
    See: https://www.sciencedirect.com/science/article/abs/pii/S0921889009001900
    """

    assert len(kernel) % 2 == 1

    distances = [dist(to_cartesian((lidar_to_rad(i), scan_points[i])), to_cartesian((lidar_to_rad(i), scan_points[i+1]))) for i in range(len(scan_points) - 1)]

    # calculate stdev
    sigma = np.std(distances)

    # half the kernel
    half_kernel = len(kernel) // 2

    clusters: list[list[tuple[float, float]]] = []

    # convolution
    # consider using vectorized np.convolve and checking for clusters after
    last = 0
    for i in range(len(distances)):
        integral = 0
        for j in range(-half_kernel, half_kernel + 1):
            if 0 <= i+j and i+j < len(distances):
                integral += distances[i+j] * kernel[j+half_kernel]
        if integral > deviations * sigma: # typo in the paper for d = 5, meant b
            clusters.append([to_cartesian((r, dist)) for r, dist in enumerate(scan_points[last:i])])
            last = i
    clusters.append(scan_points[last:])

    return clusters


def reholt(scan_points: NDArray):
    # actually who named this
    base = 1

    j = 1
    d = 0
    while d >= d1:
        j += 1
        d = dist(scan_points[base], scan_points[j])
