#import logging
from olaves_robot import OlavesRobot
#from math import pi

r = OlavesRobot()

# def configure_logging():
#     """
#     Configures logging for the entire application.
#
#     This function should be called once at the start of the program
#     to ensure consistent logging across all modules.
#     """
#     # Set the default logging level and format
#     # logging.basicConfig(
#     #   level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
#     # )


def run_robot():
    """
    This is to test if power can run in the robot.
    """
    
    r._move(10)



run_robot()