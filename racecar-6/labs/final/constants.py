from group_6.control import *
from group_6.vision import *

IS_SIMULATION = "-s" in sys.argv
IS_REAL = not IS_SIMULATION

# Colors

YELLOW: Color = ((40-20, 90, 150), (30+20, 255, 255))
BLUE: Color = ((91-20, 120-45, 206-30), (91+20, 255, 255))
GREEN: Color = ((30, 160, 150), (60+30, 255, 255+40))
ORANGE: Color = ((0, 125, 200), (16, 255, 255))
PURPLE: Color = ((100, 90, 70), (160, 255, 255))
RED: Color = ((150-20, 85-45, 190-30), (179, 255, 255))

GRAVEYARD_COLOR_PRIORITY = (YELLOW, BLUE)

# LIDAR

OFFSET = 8
RIGHT_WINDOW = (90 - OFFSET, 90)
LEFT_WINDOW = (270, 270 + OFFSET)
FRONT_WINDOW = (-OFFSET // 2, OFFSET // 2)

# PID

LINE_FOLLOW_PID = PIDConstants(
    k_p=1 if IS_REAL else 16.0,
    k_i=0,
    k_d=0.01 if IS_REAL else 0.1
)

# LANE_FOLLOW_PID = LINE_FOLLOW_PID * (1.0/2.0)

CENTER_WALL_PID = PIDConstants(
    k_p=0.0013 if IS_REAL else 0.8,
    k_i=0.0,
    k_d=0.0008 if IS_REAL else 0.1
)

RIGHT_WALL_PID = PIDConstants(
    k_p=0.0035,
    k_i=0.0,
    k_d=0.0008,
)

CONE_SLALOM_PID = PIDConstants(
    k_p=0.5 if IS_REAL else 8.0,
    k_i=0,
    k_d=0.01 if IS_REAL else 0.1
)

FOLLOWING_SPEED = 0.145 if IS_REAL else 1
WEIGHT_COMPENSATION = 0.2
