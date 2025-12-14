#Backup code which will be hardcoded - used if main code has error, or for challenges in virtual league (Note that this must work)

from typing import Optional, List, Tuple
from abc import ABC, abstractmethod
from olaves_robot import OlavesRobot
import logging

logger = logging.getLogger(__name__)

class Backup(ABC):
    def __init__(self, robot: OlavesRobot):
        self.r = robot

    def hardcode(self):
        """
        A hardcoded backup function to perform basic robot operations.
        This function can be expanded to include specific hardcoded sequences
        for the robot to execute in case of failures in the main logic.
        """
        logger.info("Executing hardcoded backup routine.")
        # Example hardcoded routine:
        self.r.move_to_marker(110)
        self.r._move(0.05)
        self.r.sleep(0.5)
        self.r._move(0.05)
        self.r.sleep(1)
        self.r.pick_hold_box()
        self.r.sleep(3)

    #TODO: Hardcode a competition strategy which doesn't require camera, and only has movement, picking and dropping boxes, and rotation by a known amount