import abc
import sys
from typing import Any, List, Tuple
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_utils as rc_utils
from group_6.control import *
from group_6.vision import *


class RobotData(NamedTuple):
    """
    Data class for important data in the robot
    """
    image: NDArray
    visible_tags: List[rc_utils.ARMarker]
    lidar_scan: NDArray

    def get_visible_ids(self):
        """
        Returns all currently visible aruco ids
        """
        return [tag.get_id() for tag in self.visible_tags]

    def __repr__(self) -> str:
        return f"Visible Tags: {self.get_visible_ids()})"


class State(abc.ABC):
    """
    Represents a possible state of the robot that takes in RobotData as input
    and outputs (speed, angle), as well as defines transitions to other states
    """

    @abc.abstractmethod
    def execute(self, data: RobotData) -> Tuple[float, float]:
        pass

    @abc.abstractmethod
    def next_state(self, data: RobotData) -> Any:
        return self


def to_steering_angle(val: float, old_min: float, old_max: float) -> float:
    """
    Remaps value from old scale to [-0.25, 0.25], the scale of our duty cycle wheels
    They use 0.25 instead of 1 due to a conversion error in the racecar library
    """
    # ali's code: ********&***&***&***&
    return rc_utils.remap_range(val, old_min, old_max, -0.25, 0.25, True)

# def stopped(_: RobotData) -> Tuple[float, float]:
#     """
#     Stopped state returns (0, 0) always
#     """
#     return (0, 0)