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
import commands

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

global queue
queue = []

global location
location = [0,0]
global angle
angle = 90

global velocity
velocity = 0

global loc_over_time
loc_over_time = []
loc_over_time.append(location)

global counter
counter = 0
def start():
    """
    This function is run once every time the start button is pressed
    """
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
    rc.drive.set_speed_angle(0, 0)
    global angle
    global location
    global velocity
    global loc_over_time
    global counter
    global scheduler
    


    accel = rc.physics.get_linear_acceleration()[2] 
    velocity += accel if accel > 0.05 or accel < -0.05 else 0
    
    location[0] += math.cos(math.radians(angle)) * velocity * rc.get_delta_time()
    location[1] += math.sin(math.radians(angle)) * velocity * rc.get_delta_time()
    loc_over_time.append(location.copy())
    angle -= math.degrees(rc.physics.get_angular_velocity()[1]) * rc.get_delta_time() 

    if rc.controller.was_pressed(rc.controller.Button.X):
        scheduler.schedule(commands.run(lambda: rc.drive.set_speed_angle(1, -.02)).with_timeout(1))
    
    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Driving in a circle...")
        queue.append([5.5,1,1])
        
    if rc.controller.was_pressed(rc.controller.Button.Y):
        print("Driving in a star...")
        for _ in range(5):
            # queue.append(("rotrel", 30))
            queue.append([5,1,0])
            queue.append([1,1,1])
            queue.append([3,-1,-1])
            # queue.append([4,1,0])
         
            # queue.append([2,-1,-1])
            # queue.append([2,-1,0])
            # queue.append([2,-1,-1])

    if rc.controller.was_pressed(rc.controller.Button.B):
        print("Driving in a square...")
        for _ in range(4):
            queue.append([2,0.5,0])
            queue.append([0.5,0.5,1])

    # TODO (main challenge): Drive in a square when the B button is pressed

    # TODO (main challenge): Drive in a figure eight when the X button is pressed

    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # is pressed

     
    if len(queue) != 0:
        # if queue[0][0] == "rotrel":
        #     start_angular_vel = rc.physics.get_angular_velocity()
        #     queue.pop(0)
        # else:
            queue[0][0] -= rc.get_delta_time()
            command = queue[0]

            rc.drive.set_speed_angle(command[1], command[2])
            if command[0] <= 0:
                x = np.array(loc_over_time) 
                print(x)
                plt.scatter(x[:, 0], x[:, 1])
                plt.savefig('foo.png')
                velocity = 0
                loc_over_time = []
                loc_over_time.append(location)
                queue.pop(0)

    scheduler.run()
            
        

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
