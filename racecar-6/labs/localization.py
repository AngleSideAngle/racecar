
import sys
import math

from typing import Callable, NamedTuple, Optional, Tuple

import cv2
import numpy as np
from nptyping import NDArray

K_1 = np.array((-3, -3, 5, -3, -3))
K_2 = [-1, -2, -3, 5, -3, -2, -1]

def dist(p1: Tuple[float, float], p2: Tuple[float, float]):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def dcc(scan_points: NDArray, kernel: NDArray = K_1, d: float = 5):
    assert len(kernel) % 2 == 0

    distances = [dist(scan_points[i], scan_points[i+1]) for i in range(len(scan_points) - 1)]
    
    sigma = 

    # half the kernel
    m = len(kernel) // 2

    clusters = []

    # convolution
    # consider using vectorized np.convolve and checking for clusters after
    last = 0
    for i in range(len(distances)):
        integral = 0
        for j in range(-m, m + 1):
            integral += distances[i+j] * kernel[j+m]
        if integral > d * sigma: # b * sigma ?? typo for d = 5
            clusters.append(distances[last:i])
            last = i

    return clusters


def reholt(scan_points: NDArray):
    # actually who named this
    base = 1

    j = 1
    d = 0
    while d >= d1:
        j += 1
        d = dist(scan_points[base], scan_points[j])