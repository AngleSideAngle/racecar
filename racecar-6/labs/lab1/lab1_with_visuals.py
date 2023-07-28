"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################
import matplotlib.pyplot as plt
import sys
import math
import numpy as np

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
import group_6.commands as commands

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here

########################################################################################
# Functions
########################################################################################

global scheduler
scheduler: commands.Scheduler = commands.Scheduler(default_command=commands.run(rc.drive.stop))

global location
location = [0,0]
global angle
angle = 90

global velocity
velocity = 0

global loc_over_time
loc_over_time = []
loc_over_time.append(location)

def drive_command(time: float, power: float, angle: float) -> commands.Command:
    return commands.run(lambda: rc.drive.set_speed_angle(power, angle)).and_then(commands.run(rc.drive.stop)).with_timeout(time)

def square() -> commands.Command:
    side = lambda: commands.Sequence(
        drive_command(1, 0.3, 0),
        drive_command(2, 0.2, 1)
    )
    return commands.Sequence(*[side() for _ in range(4)]).along_with(commands.print_once("Driving in a square..."))

def star() -> commands.Command:
    side = lambda: commands.Sequence(
        drive_command(5, 1, 0),
        drive_command(1, 1, 1),
        drive_command(3, -1, -1)
    )
    return commands.Sequence(*[side() for _ in range(5)]).along_with(commands.print_once("Driving in a star..."))

def circle(angle: float = 1) -> commands.Command:
    return drive_command(5.5, 1, angle).along_with(commands.print_once("Driving in a circle..."))

def eight() -> commands.Command:
    return circle(angle=1).and_then(circle(angle=-1))
    

def start():
    """
    This function is run once every time the start button is pressed
    """
    a = circle()
    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO (warmup): Implement acceleration and steering
    global angle
    global location
    global velocity
    global loc_over_time

    accel = rc.physics.get_linear_acceleration()[2] 
    velocity += accel if accel > 0.05 or accel < -0.05 else 0
    
    location[0] += math.cos(math.radians(angle)) * velocity * rc.get_delta_time()
    location[1] += math.sin(math.radians(angle)) * velocity * rc.get_delta_time()
    loc_over_time.append(location.copy())
    angle -= math.degrees(rc.physics.get_angular_velocity()[1]) * rc.get_delta_time() 
    
    if rc.controller.was_pressed(rc.controller.Button.A):
        scheduler.schedule(circle())

    if rc.controller.was_pressed(rc.controller.Button.Y):
        scheduler.schedule(star())

    if rc.controller.was_pressed(rc.controller.Button.X):
        scheduler.schedule(eight())

    if rc.controller.was_pressed(rc.controller.Button.B):
        scheduler.schedule(square())

    # TODO (main challenge): Drive in a square when the B button is pressed

    # TODO (main challenge): Drive in a figure eight when the X button is pressed

    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # is pressed

    scheduler.run()
            
        

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
