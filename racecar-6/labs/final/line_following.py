import sys
from typing import Tuple

sys.path.insert(0, "../../library")
from group_6.control import *
from group_6.vision import *
from group_6.utils import *
from constants import *
from wall_following import SideWallFollowing

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

        self.contour = get_contour(data.image, self.color_priority, crop_floor, min_contour_area=60)

        in_sight = self.debouncer.update(self.contour is not None)

        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if self.contour is not None:
            # print(f"Color: {contour.color}")
            angular_offset = to_steering_angle(self.contour.center[1], 0, data.image.shape[1])
            angle = self.controller.calculate(position=0, setpoint=angular_offset)

        if in_sight:
            speed = FOLLOWING_SPEED
        else:
            # print("reversing")
            speed = -FOLLOWING_SPEED

        return (speed, angle)
    
    def next_state(self, data: RobotData) -> Any:
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

        if 3 in data.get_visible_ids(): # canyon maze
            return SideWallFollowing(right_wall=True)

        return self