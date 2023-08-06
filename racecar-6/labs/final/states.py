import sys
from typing import Any, List, Tuple
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from group_6.control import *
from group_6.vision import *
from group_6.localization import *
from group_6.utils import *
from constants import *

class LineFollowing(State):

    controller = PIDController(
        LINE_FOLLOW_PID,
        min_output=-1,
        max_output=1
    )

    debouncer = Debouncer(baseline=True, debounce_time=0.3)

    contour: Optional[ContourData]

    def __init__(self, color_priority) -> None:
        self.color_priority = color_priority

    def execute(self, data: RobotData) -> Tuple[float, float]:
        speed = 0
        angle = 0

        self.contour = get_contour(data.image, self.color_priority, crop_floor, min_contour_area=25)

        in_sight = self.debouncer.update(self.contour is not None)

        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if self.contour is not None:
            # print(f"Color: {contour.color}")
            angular_offset = to_steering_angle(self.contour.center[1], 0, data.image.shape[1])
            angle = self.controller.calculate(position=0, setpoint=angular_offset)

            speed = FOLLOWING_SPEED
        # if in_sight:
        #     speed = FOLLOWING_SPEED
        # else:
        #     # print("reversing")
        #     speed = -FOLLOWING_SPEED

        return (speed, angle)
    
    def next_state(self, data: RobotData) -> Any:
        if 3 in data.get_visible_ids(): # canyon maze
            return SideWallFollowing(right_wall=True)

        return self

    def __repr__(self) -> str:
        return f"Line Following: Contour={self.contour}"

class LaneFollowing(State):

    controller = PIDController(
        LINE_FOLLOW_PID,
        setpoint=0,
        min_output=-1,
        max_output=1
    )

    def __init__(self, color: Color) -> None:
        self.color = color

    def execute(self, data: RobotData) -> Tuple[float, float]:
        angle = 0
        speed = FOLLOWING_SPEED

        left = get_contour(data.image, (self.color, ), cropper=crop_left, min_contour_area=180)
        right = get_contour(data.image, (self.color, ), cropper=crop_right, min_contour_area=180)

        left_dist = left.bounds[1] - left.center[1] if left is not None else data.image.shape[1] // 4
        right_dist = right.bounds[1] if right is not None else data.image.shape[1] // 4

        position = to_steering_angle(left_dist-right_dist, -data.image.shape[1] / 2, data.image.shape[1] / 2)

        angle = self.controller.calculate(position=position)

        return (speed, angle)
    
    def next_state(self, data: RobotData) -> Any:
        return self


class ConeSlalom(State):

    # cone slalom
    right_cone: Color
    left_cone: Color
    default_angle: float

    controller = PIDController(
        CONE_SLALOM_PID,
        min_output=-1,
        max_output=1
    )

    debouncer = Debouncer(baseline=True, debounce_time=0.3)
    angle_limiter = RateLimiter(rate=0.05)

    prev_cone: Optional[Color] = None
    target_cone: Color

    seen_cones = 0
    CONE_THRESHOLD = 4

    def __init__(
        self,
        right_color: Color = ORANGE,
        left_color: Color = PURPLE,
        default_angle: float = 0.14
    ) -> None:
        self.right_cone = right_color
        self.left_cone = left_color
        self.default_angle = default_angle

        self.target_cone = self.right_cone

    def execute(self, data: RobotData) -> Tuple[float, float]:
        angle = 0

        targets = (self.target_cone, self.prev_cone) if self.prev_cone is not None else (self.target_cone, )
        cone = get_contour(data.image, targets, crop_bottom_3_4ths, min_contour_area=100)

        # print(f"prev cone: {self.prev_cone}, target cone: {self.target_cone}")

        in_sight = self.debouncer.update(cone is not None)

        if cone is not None:
            self.prev_cone = cone.color

            # print(f"cone color: {cone.color}")
            color_offset = cone.area / data.image.shape[1] / data.image.shape[0] * (-1 if cone.color == self.left_cone else 1)

            center = rc_utils.remap_range(cone.center[1], 0, data.image.shape[1], -1, 1, True)
            cone_offset = min(center, 0) if cone.color == self.left_cone else max(center, 0)

            # setpoint = to_steering_angle(cone_offset, 0, image.shape[1]) + color_offset

            angle = self.controller.calculate(position=0, setpoint=color_offset+cone_offset)
            # print(f"going to {cone.color}")
        elif self.prev_cone is not None and not in_sight:
            self.target_cone = self.right_cone if self.prev_cone == self.left_cone else self.left_cone
            # print(f"SWITCHING TO: {self.target_cone}")
            limited_angle = self.default_angle * (1 if self.target_cone == self.right_cone else -1)
            angle = limited_angle

        return (FOLLOWING_SPEED, angle)
    
    def next_state(self, data: RobotData) -> Any:
        return self

    def __repr__(self) -> str:
        return f"{self.__class__}: {self.__dict__}"


class SideWallFollowing(State):

    controller = PIDController(
        RIGHT_WALL_PID,
        min_output=-1,
        max_output=1
    )

    def __init__(self, right_wall: bool, speed: float = SIDE_FOLLOWING_SPEED, offset: float = 70) -> None:
        self.controller.setpoint = offset
        self.angle = 55 if right_wall else 305
        self.speed = speed

    def execute(self, data: RobotData) -> Tuple[float, float]:

        # front_dist = rc_utils.get_lidar_average_distance(data.lidar_scan, 0)

        dist = rc_utils.get_lidar_average_distance(data.lidar_scan, self.angle, 35)
        front = rc_utils.get_lidar_average_distance(data.lidar_scan, 0, 2)

        angle = self.controller.calculate(position=dist) * (-1 if self.angle == 55 else 1)

        if front < 50:
            angle = -1 if self.angle == 55 else 1

        return (self.speed, angle)

    def next_state(self, data: RobotData) -> Any:
        if 4 in data.get_visible_ids(): # go fast speedway
            return CenterWallFollowing(speed=FOLLOWING_SPEED+0.02)

        if 8 in data.get_visible_ids(): # brick walls
            return SideWallFollowing(right_wall=True)

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

        right_dist = rc_utils.get_lidar_average_distance(data.lidar_scan, 55, 35)
        left_dist = rc_utils.get_lidar_average_distance(data.lidar_scan, 305, 35)
        # _, front_dist = rc_utils.get_lidar_closest_point(data.lidar_scan, FRONT_WINDOW)

        # if left_dist < right_dist:
        #     angle = self.left_controller.calculate(position=-left_dist)
        # else:
        #     angle = self.right_controller.calculate(position=right_dist)

        angle = self.controller.calculate(position=left_dist-right_dist)

        # if front_dist < 100:
        #     speed -= 0.05

        # return (speed - 0.01 + abs(angle) / 20, angle)
        return (FOLLOWING_SPEED, angle)

    def next_state(self, data: RobotData) -> Any:
        if 1 in data.get_visible_ids(): # overpass
            pass

        if 2 in data.get_visible_ids(): # graveyard
            # return LineFollowing(GRAVEYARD_COLOR_PRIORITY)
            return SideWallFollowing(right_wall=True)
        
        # if 3 in data.get_visible_ids(): # canyon maze
        #     return SideWallFollowing(right_wall=True)
        
        if 5 in data.get_visible_ids():
            self.speed = FOLLOWING_SPEED - 0.01
            return self
        
        if 6 in data.get_visible_ids(): # green line following
            # return LineFollowing((GREEN, ))
            return SideWallFollowing(right_wall=False)

        return self

    def __repr__(self) -> str:
        return f"Wall Following (Center): {self.controller}"