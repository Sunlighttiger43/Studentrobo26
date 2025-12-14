# robot.py

import logging
from states import main
from olaves_robot import OlavesRobot
from Backup import hardcode

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
    """

    try:
        main()
    except Exception:
        hardcode()

    # configure_logging()
    # # Call main from states
    # main()


run_robot()