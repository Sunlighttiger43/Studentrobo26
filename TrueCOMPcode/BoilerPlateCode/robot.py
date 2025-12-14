
# robot.py
"""
We will now test the main low level functions for our robot.
This involves move_to_marker, pick_hold_box, drop_box and get_marker_by_id

- First test involves simply searching for the box, and picking it up, just to drop it quickly after
- You can then test it without the box initially in sight
"""


import logging
from olaves_robot import OlavesRobot
from math import pi

r = OlavesRobot()

def configure_logging():
    """
    Configures logging for the entire application.

    This function should be called once at the start of the program
    to ensure consistent logging across all modules.
    """
    # Set the default logging level and format
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )


def run_robot():
    """
    Main entry point for the robot program.

    This function configures logging and then starts the main state machine.

    Initial Settings:
        - Just above the endstops
        - Cage fully open
    """
    while True:
        r.move_to_marker(110)
        r._move(0.05)
        r.sleep(0.5)
        r._move(0.05)
        r.sleep(1)
        r.pick_hold_box()
        r.sleep(3)



run_robot()
