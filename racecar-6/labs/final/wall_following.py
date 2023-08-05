import sys
from typing import Tuple

sys.path.insert(0, "../../library")
import racecar_utils as rc_utils
from group_6.control import *
from group_6.vision import *
from group_6.utils import *
from group_6.localization import *
from constants import *
from line_following import LineFollowing

class SideWallFollowing(State):

    controller = PIDController(
        RIGHT_WALL_PID,
        min_output=-1,
        max_output=1
    )

    def __init__(self, right_wall: bool, offset: float = 60) -> None:
        self.controller.setpoint = offset
        self.angle = 90 if right_wall else 270

    def execute(self, data: RobotData) -> Tuple[float, float]:

        # front_dist = rc_utils.get_lidar_average_distance(data.lidar_scan, 0)

        dist = rc_utils.get_lidar_average_distance(data.lidar_scan, self.angle)

        angle = self.controller.calculate(position=dist) * (-1 if self.angle == 90 else 1)

        return (FOLLOWING_SPEED, angle)
    
    def next_state(self, data: RobotData) -> Any:
        if 4 in data.get_visible_ids(): # go fast speedway
            return CenterWallFollowing()

        return self

class CenterWallFollowing(State):

    controller = PIDController(
        CENTER_WALL_PID,
        min_output=-1,
        max_output=1
    )

    def __init__(self, speed: float = FOLLOWING_SPEED) -> None:
        self.speed = speed

    def execute(self, data: RobotData) -> Tuple[float, float]:
        # speed = FOLLOWING_SPEED

        right_dist = rc_utils.get_lidar_average_distance(data.lidar_scan, 45, 35)
        left_dist = rc_utils.get_lidar_average_distance(data.lidar_scan, 315, 35)
        # _, front_dist = rc_utils.get_lidar_closest_point(data.lidar_scan, FRONT_WINDOW)

        # if left_dist < right_dist:
        #     angle = self.left_controller.calculate(position=-left_dist)
        # else:
        #     angle = self.right_controller.calculate(position=right_dist)

        angle = self.controller.calculate(position=left_dist-right_dist)

        # if front_dist < 100:
        #     speed -= 0.05

        # return (speed - 0.01 + abs(angle) / 20, angle)
        return (self.speed, angle)
    
    def next_state(self, data: RobotData) -> Any:
        if 1 in data.get_visible_ids(): # overpass
            pass

        if 2 in data.get_visible_ids(): # graveyard
            return LineFollowing(GRAVEYARD_COLOR_PRIORITY)

        return self

    def __repr__(self) -> str:
        return f"Wall Following (Center): {self.controller}"