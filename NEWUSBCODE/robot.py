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

def run_robot():
    """
    This is to test if power can run in the robot.
    """
    #wallMarkers = r.WallMarkerzone(r.zone)
    get_AcidBlock()
    r._turn(pi)
    r.sleep(3)
    go_to_wallmarker()
    
      

def get_AcidBlock() -> None:
    """
    This is to identify the acid blocks.
    """
    #acid = r.get_marker_by_idrange(r.types_of_markers["acidic"])
    #if len(acid)>1:
    #    acid = acid[0]
    r.move_to_marker(110)
    r.pick_hold_box()

def go_to_wallmarker() -> None:
    '''
    This is for the robot to move to the wall marker and drop the acid block.
    '''
    #wall = r.get_marker_by_idrange(r.types_of_markers["wall"])
    #wall = []
    #if len(wall)>1:
    #    wall = wall[0]
    r.move_to_marker(194)
    r.drop_box()

run_robot()
