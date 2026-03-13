# robot.py
"""
We will now test the main low level functions for our robot.
This involves move_to_marker, pick_hold_box, drop_box and get_marker_by_id

- First test involves simply searching for the box, and picking it up, just to drop it quickly after
- You can then test it without the box initially in sight
"""
import math
from math import pi, sin, cos, atan2, sqrt
from time import time, struct_time
from typing import List, Tuple, Optional, Union
from pid import PIDController
from zone_coords import zone_coords

import numpy as np
import sr.robot3 as sr

import logging
import json
import os

#from states import main
from sr.robot3 import Robot


rob = Robot()
my_arduino = rob.arduinos["75230313833351616121"]

my_motor_board = rob.motor_board

def motor_right_speed(pwr):
	mult = 1
	my_motor_board.motors[0].power = pwr * mult

def motor_left_speed(pwr):
	mult = 0.933
	my_motor_board.motors[1].power = pwr * mult

def forward_move():
	motor_left_speed(1)
	motor_right_speed(1)

	rob.sleep(3)

	motor_left_speed(0)
	motor_right_speed(0)


	rob.sleep(2)

def turn_left():
	motor_right_speed(1)
	motor_left_speed(-1)

	rob.sleep(0.528)

	# motor_left_speed(1)
	# motor_right_speed(1)

	# rob.sleep(3)

	motor_left_speed(0)
	motor_right_speed(0)

	rob.sleep(3)

def full_left_turn():
	motor_right_speed(1)
	motor_left_speed(-1)

	rob.sleep(2.12)

	motor_left_speed(0)
	motor_right_speed(0)

	rob.sleep(3)


def hardward_initialisation() -> None:
        stepperuino_write(data="@")
        rob.sleep(3)

        stepperuino_write(data="u")
        rob.sleep(3)

        for i in range(8):
                stepperuino_write(data="-")
                rob.sleep(3)
        
def stepperuino_write(data: str) -> None:
        my_arduino.command(data)

def pick_hold_box():
        """
        Picks up and holds the box.

        Assumes that the box is directly ahead of the robot.

        This method ensures the robot returns to its initial starting point and engages the servo to pick up the box.
        It activates the power board to secure the box and logs the action.

        
        """

        for i in range(8):
            stepperuino_write(data="+")
        rob.sleep(3)

        stepperuino_write(data="d")
        rob.sleep(3)

        for i in range(8):
            stepperuino_write(data="-")
        rob.sleep(3)

        stepperuino_write(data="u")
        rob.sleep(3)

#hardward_initialisation()
forward_move()
stepperuino_write(data="+")
pick_hold_box()
forward_move()
