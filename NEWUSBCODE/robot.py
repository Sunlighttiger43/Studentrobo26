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
    This is to test if power can run in the robot.
    """
    #wallMarkers = r.WallMarkerzone(r.zone)
    while True:
        get_AcidBlock()
        r._turn(pi/4)
        go_to_wallmarker()
        r._move(-0.5)
        r._turn(pi*1.5)
    
    
    

def get_AcidBlock() -> None:
    """
    This is to identify the acid blocks.
    """
    found_acid = False
    ignore_block = []
    ignore_count = 0
    target_markers = r.types_of_markers["basic"]
    
    while not found_acid:
        acid = r.get_marker_by_idrange(r.types_of_markers["basic"])
        print(acid)
        while not acid:
            if acid is None:
                r._move(-0.15)
                break
            else:
                ang = acid[0].orientation.pitch
                dist = acid[0].position.distance
                vert_height = dist * sin(ang)
                #r.logger.info(f"[get_AcidBlock] | Reject {acid[0]} with height {vert_height}")
                if  vert_height >= 30:
                    acid.pop(0)
                else:
                    break
                
        if acid is not None:
            if len(acid)>1:
                acid = acid[0]
            


            found_acid = r.move_to_marker(acid)
            
            if found_acid:
                ignore_count = 0
            else:
                ignore_count += 1

            if ignore_count >= 4:
                try:
                    target_markers = target_markers.remove(acid)
                except:
                    target_markers = target_markers

    
    r.pick_hold_box()
    r._move(-0.3)

def go_to_wallmarker() -> None:
    '''
    This is for the robot to move to the wall marker and drop the acid block.
    '''
    home = None

    while not home:
        home = r.get_marker_by_idrange(r.WallMarkerzone(r.zone))
        if not home:
            r._move(-0.3)
            r._turn(pi/8)
            r._move(2)
            r._turn(pi/4)

    if len(home)>1:
        home = home[0]
    r.move_to_marker(home)
    r.drop_box()

run_robot()
