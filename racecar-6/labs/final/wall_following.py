import sys
from typing import Tuple

sys.path.insert(0, "../../library")
import racecar_utils as rc_utils
from group_6.control import *
from group_6.vision import *
from group_6.utils import *
from constants import *

class RightWallFollowing:

    controller = PIDController(
        RIGHT_WALL_PID,
        min_output=-1,
        max_output=1
    )

    def __init__(self, offset: float = 70) -> None:
        self.controller.setpoint = offset


    def __call__(self, data: RobotData) -> Tuple[float, float]:
        _, right_dist = rc_utils.get_lidar_closest_point(data.scan, RIGHT_WINDOW)

        angle = -self.controller.calculate(position=right_dist)

        return (FOLLOWING_SPEED, angle)

class CenterWallFollowing:

    controller = PIDController(
        CENTER_WALL_PID,
        min_output=-1,
        max_output=1
    )

    def __call__(self, data: RobotData) -> Tuple[float, float]:

        _, left_dist = rc_utils.get_lidar_closest_point(data.scan, LEFT_WINDOW)
        _, right_dist = rc_utils.get_lidar_closest_point(data.scan, RIGHT_WINDOW)

        angle = self.controller.calculate(position=left_dist-right_dist)

        return (FOLLOWING_SPEED, angle)

    def __repr__(self) -> str:
        return f"Wall Following (Center): {self.controller}"