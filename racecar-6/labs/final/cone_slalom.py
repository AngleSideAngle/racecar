import sys
from typing import Any, Tuple
from group_6.utils import RobotData

sys.path.insert(0, "../../library")
import racecar_utils as rc_utils
from group_6.control import *
from group_6.vision import *
from group_6.utils import *
from constants import *


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
