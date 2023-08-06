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

import time
import math

from typing import Tuple, List
from collections import namedtuple

import numpy as np
from nptyping import NDArray

K_1 = np.array((-3, -3, 5, -3, -3))
K_2 = np.array((-1, -2, -3, 5, -3, -2, -1))

Line = namedtuple("Line", ["point_1", "point_2"])

class ParticleFilter:
    pass

class IMUOdometry:
    """
    Performs dead reckoning via filtering, integrating, and rotatinglinear acceleration and angular
    velocity
    """

    # GRAVITY: NDArray = np.array((0.0, -9.81, 0.0))

    def __init__(self, moving_avg_len: int = 7) -> None:
        self.position = np.array((0.0, 0.0, 0.0))
        self.velocity = np.array((0.0, 0.0, 0.0))
        self.acceleration = np.array((0.0, 0.0, 0.0))

        self.angular_velocity = np.array((0.0, 0.0, 0.0))
        self.angular_position = np.array((0.0, 0.0, 0.0))

        self.prev_time = time.perf_counter()

        self.moving_avg = [np.array((0, 0, 0)) for _ in range(moving_avg_len)]

    def update_odometry(self, linear_acceleration: NDArray, angular_velocity: NDArray) -> None:
        """
        Updates odometry based on imu data
        """

        current_time = time.perf_counter()
        delta_time = current_time - self.prev_time
        self.prev_time = current_time

        self.angular_velocity = angular_velocity
        self.angular_position += angular_velocity * delta_time

        self.moving_avg.pop(0)
        self.moving_avg.append(linear_acceleration)

        self.acceleration = sum(self.moving_avg) / len(self.moving_avg)
        self.velocity += self.acceleration * delta_time
        self.position += self.velocity * delta_time


def dist(p_1: Tuple[float, float], p_2: Tuple[float, float]):
    """
    Distance between two cartiesian points
    """
    return math.sqrt((p_1[0] - p_2[0]) ** 2 + (p_1[1] - p_2[1]) ** 2)

def dist_from_line(point: Tuple[float, float], line: Line) -> float:
    """
    Length of a perpendicular bisector to the line that intersects with the point
    """
    return np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) / np.linalg.norm(line[1] - line[0])

def polar_to_cartesian(polar: Tuple[float, float]) -> Tuple[float, float]:
    """
    Converts polar coordinates to cartesian
    """
    return (polar[0] * math.cos(polar[1]), polar[0] * math.sin(polar[1]))

def lidar_to_rad(index: int) -> float:
    """
    Converts lidar index value (1/2 deg) to radians
    """
    return index / 2.0 * math.pi / 180.0

def dcc(scan_points: NDArray, kernel: NDArray = K_1, deviations: float = 5) -> List[NDArray]:
    """
    Performs Distance-based Convolution Clustering on a set of scan points
    See: https://www.sciencedirect.com/science/article/abs/pii/S0921889009001900
    """

    assert len(kernel) % 2 == 1

    def lidar_to_cartesian(index: int) -> Tuple[float, float]:
        """
        Utility to convert lidar values into cartesian points
        """
        return polar_to_cartesian((scan_points[index], lidar_to_rad(index)))

    # this could be vectorized
    distances = np.array([dist(lidar_to_cartesian(i), lidar_to_cartesian(i+1)) for i in range(len(scan_points) - 1)])

    # calculate stdev
    sigma = np.std(distances)

    # half the kernel
    half_kernel = len(kernel) // 2

    clusters: List[NDArray] = []

    # convolution
    # consider using vectorized np.convolve and checking for clusters after
    last = 0
    for i in range(len(distances)):
        integral = 0
        for j in range(-half_kernel, half_kernel + 1):
            if 0 <= i+j and i+j < len(distances):
                integral += distances[i+j] * kernel[j+half_kernel]
        if integral > deviations * sigma: # typo in the paper for d = 5, meant b
            clusters.append(np.array([lidar_to_cartesian(theta) for theta in range(last, i)])) # type: ignore
            last = i+1
    clusters.append(np.array([lidar_to_cartesian(theta) for theta in range (last, len(scan_points))])) # type: ignore

    return clusters


def rht(scan_points: NDArray):
    """
    Performs Reduced Hough Transform (REHOLT) on a cluster of points to identify lines
    scan_points: ndarray of cartesian data points
    """

    D1 = None
    D2 = None

    base = 0
    j = base

    d = 0
    while True:
        j += 1
        d = dist(scan_points[base], scan_points[j])
        if d >= D1:
            break

    R = Line(scan_points[base], scan_points[j])
    k = j

    while k < len(scan_points):
        d = dist_from_line(scan_points[k+1], R)
        if d > D2:
            new_line = rht(scan_points[base:k])
            # p_b = closest_point_from_line(scan_points[k], new_line) <- unclear why this is done
        else:
            k += 1

    return R


def pht(scan_points: NDArray):
    """
    Performs a Probabilistic Hough Transform
    https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/AV1011/macdonald.pdf
    """

    pass