# states

from typing import Optional, List, Tuple
from abc import ABC, abstractmethod
from olaves_robot import OlavesRobot
import logging

logger = logging.getLogger(__name__)

class State(ABC):
    def __init__(self, robot: OlavesRobot):
        self.r = robot

class StateMachine:
    def __init__(self, initial_state: State):
        self.initial_state = initial_state

    def run(self) -> None:
        """
        Starts the state machine execution loop.

        IMPORTANT: This function must be called to start running the first state.
        """
        state = self.initial_state
        while state is not None:
            logger.info(f"Running state: {type(state).__name__}")
            state = state.run()
        logger.info("State machine execution completed.")


class MainLoop(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)

    def _calculate_intensity(self) -> int:
        """
        Calculates the current intensity based on time elapsed and number of pallets.

        Returns:
            int: The calculated intensity value.

        Notes:
            - Intensity increases over time and with the number of pallets.
            - This function can be extended to include more factors influencing intensity.
        """
        elapsed_time = self.r.get_comp_time()
        number_of_pallets = self._get_number_of_pallets()
        boxes_left = 6 - self._get_number_of_pallets
        
        if elapsed_time == 150:
            intensity = 3
        elif boxes_left == 2 and elapsed_time >= 135 or boxes_left == 1 and elapsed_time >= 140 or boxes_left == 0:
            intensity = 2
        elif boxes_left >= 2 :
            intensity = 1
        """
        we remoddelled the intensity so that it will keep hunting until 2 boxes 
        it will switch to the knocking state if we have 15 seconds left with 2 boxes or 1 box left with 10 seconds or we have 0 boxes left
        if the time_elapsed is past the total round time, it will switch to sleep state
        """            

         # EXAMPLE INTENSITY CALCULATION
        # TODO: Factor in number of pallets into the calculation
        logger.debug(f"Calculated intensity: {intensity} (Elapsed Time: {elapsed_time}, Pallettes: {number_of_pallets})")
        return intensity
    
    def _get_number_of_pallets(self) -> int:
        """
        Retrieves the current number of our pallets on highrises

        Returns:
            int: The number of pallets detected.

        Notes:
            - This function should be implemented to accurately count the number of pallets.
            - It can utilise sensors or camera inputs to determine pallet count.
        """

        # TODO: Implement this function to return a meaningful number to be used in intensity calculations
        # Example Implementation:
        # detected_pallets = self.r.detect_pallets() # TODO: Implement detect_pallets() in olaves_robot
        detected_pallets = self.r.detect_pallets()[0]
        pallet_count = len(detected_pallets)
        logger.debug(f"Number of pallets detected: {pallet_count}")
        return pallet_count
    
    def run(self) -> Optional[State]:
        """
        Executes the main loop logic to calculate intensity and determine the next state.

        Returns:
            Optional[State]: The next state based on intensity, or None to terminate.
        """

        intensity = self._calculate_intensity()
        logger.info(f"Current Intensity: {intensity}")

        # TODO: Modify the decision statements to reflect any changes in the intensity calculation
        # Add any other states if they are needed

        if intensity == 1:
            logger.info("Transitioning to Hunting state.")
            return Hunting(self.r)
        elif intensity == 2:
            logger.info("Transitioning to Knocking state.")
            return Knocking(self.r)
        elif intensity == 3:
            logger.info("Intensity threshold reached. Terminating.")
            return None

class Hunting(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)
        self.hunting_start_time = self.r.get_comp_time()
    
    #Idea is to get marker distance as well as ID and make judgments based upon it
    def get_our_markers(self) -> List[Tuple[int, float]]:
        """
        Retrieves a list of own pallet markers.

        Returns:
            List[Tuple[int, float]]: A list of tuples containing marker IDs and their distances.
        """
        markers = self.r.camera.see()
        our_markers = []
        for marker in markers:
            # TODO: Change the implementation to account for the fact that our robot could be in any of the 4 zones
            if 100 <= marker.id <= 119:
                our_markers.append((marker.id, marker.position.distance))
        logger.debug(f"Our markers: {our_markers}")
        return our_markers

    def judgement_of_distances(self, our_markers: List[Tuple[int, float]]) -> int:
        """
        Determines the closest marker from the list of own markers.

        Parameters:
            our_markers (List[Tuple[int, float]]): A list of (ID, distance) pairs.

        Returns:
            int: The ID of the closest marker.

        Raises:
            ValueError: If 'our_markers' is empty.
        """
        if not our_markers:
            logger.error("No own markers detected for distance judgement.")
            raise ValueError("our_markers list is empty.")

        sorted_markers = sorted(our_markers, key=lambda x: x[1])
        closest_marker_id = sorted_markers[0][0]
        logger.debug(f"Closest marker ID: {closest_marker_id}")
        return closest_marker_id

    def get_hunting_time(self) -> float:
        """
        Calculates the time elapsed since the Hunting state was initiated.

        Returns:
            float: Elapsed time in seconds.
        """
        current_time = self.robot.get_comp_time()
        hunting_duration = current_time - self.hunting_start_time
        logger.debug(f"Hunting duration: {hunting_duration} seconds.")
        return hunting_duration
    
    def run(self) -> Optional[State]:
        """
        Executes the hunting state to manage the robot's own pallets.

        Returns:
            Optional[State]: The next state after completing hunting (will be the mainloop), or None to terminate.
        """
        try:
            our_markers = self.get_our_markers()
            closest_marker_id = self.judgement_of_distances(our_markers)
            logger.info(f"Moving to closest marker ID: {closest_marker_id}")
            self.robot.move_to_marker(closest_marker_id)
            # TODO: Add logic to handle how the marker is dealt with once approached
        except ValueError:
            logger.warning("No own markers detected. Returning to MainLoop.")
            # TODO: Add interrupt logic here, if no markers are detected maybe a counter should start, 
            #       if the hunting state runs some number of times in a row with no markers found, then another state should be 
            #       run instead
            return MainLoop(self.robot)


class Knocking(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)
        self.knocking_start_time = robot.get_comp_time()

    # TODO: Actually implementing the knocking state based off of the pseudocode below
    def enemy_boxes(self):
        """
        Here I am trying to identify which enemy boxes it can see which are on high rises
        :return: list[enemy_boxes]
        """
        print(f"Returning list of enemy markers")
        enemy_markers = []
        markers = self.r.camera.see()
        for marker in markers():
            type_of_marker = OlavesRobot.get_marker_type(marker)
            if type_of_marker is None and marker.orientation.pitch > 0:
                enemy_markers.append(marker)

        # So that we are not perpetually in the knocking state if we can't see any boxes to knock
        if not enemy_markers:
            return None

        return enemy_markers

    def enemy_target(self):
        """
        The actual judgement on which box we are knocking over (ie targeting) - simplest thing being implemented is whichever is closest
        :return: marker_id of box to be knocked over
        """
        enemy_markers = self.enemy_boxes()
        print(f"Returning which box to knock over")
        shortest_distance = enemy_markers[0].position.distance
        marker = enemy_markers[0]
        # Simple method to find the shortest distance
        for i in range(len(enemy_markers)):
            distance = enemy_markers[i].position.distance
            if shortest_distance > distance:
                shortest_distance = enemy_markers[i].position.distance
                marker = enemy_markers[i]

        return marker

    def knocking_over(self):
        """
        actually knocks over box with given marker_id input
        :return: If the knocking of the boxes was successful
        """
        enemy_marker=self.enemy_target()
        marker_pos=self.get_marker_pos(enemy_marker)
        self.move_to_marker(enemy_marker)
        self._move(0.8)
        pass


def main() -> None:
    """
    Main function to initialise the robot and start the state machine.

    Initialises the OlavesRobot and starts the state machine with the MainLoop state.
    """
    robot = OlavesRobot()
    logger.info("Robot initialised and starting state machine.")
    state_machine = StateMachine(MainLoop(robot))
    state_machine.run()

'''SKELETON CODE (blueprint) FOR KNOCKING STATE

the robot has to find opponent boxes to push over
this can be in a systematic way: 1. searching for opponent box stacks, from which the top box doesn't have the id of the robots own boxes 
2. knocking them over

PSEUDOCODE FOR SEARCHING ALGORITHM:
Define robot_own_ids as a list of IDs belonging to the robot
Define detected_boxes as an empty list

Function scan_for_boxes():
    Use sensors to detect all boxes in the arena
    Return a list of boxes with their positions and IDs

Function move_to_position(position):
    Navigate the robot to the specified position

Function knock_over_stack(position):
    Perform the action to knock over a stack at the given position

While True:
    detected_boxes = scan_for_boxes()

    If detected_boxes is empty:
        Search the arena systematically (e.g., follow a grid pattern)

    For each box in detected_boxes:
        If top box ID is NOT in robot_own_ids:
            move_to_position(box.position)
            knock_over_stack(box.position)
            Break out of loop
    End For
End While

CODE ITSELF:

# List of IDs belonging to the robot
robot_own_ids = [101, 102, 103]  # Example IDs for the robot's own boxes

def scan_for_boxes():
    """
    Simulate scanning for boxes in the arena.
    Returns:
        A list of detected boxes, where each box is represented as a dictionary 
        with its position (x, y) and ID.
    Example Output:
        [{"position": (5, 10), "id": 201}, {"position": (10, 15), "id": 102}]
    """
    # Replace this simulated output with actual sensor integration
    return [{"position": (5, 10), "id": 201}, {"position": (10, 15), "id": 102}]

def move_to_position(position):
    """
    Move the robot to the specified position.
    Args:
        position (tuple): The (x, y) coordinates to move to.
    """
    print(f"Moving to position {position}")
    # Replace this print statement with actual navigation code

def knock_over_stack(position):
    """
    Knock over the stack of boxes at the specified position.
    Args:
        position (tuple): The (x, y) coordinates of the stack to knock over.
    """
    print(f"Knocking over stack at {position}")
    # Replace this print statement with the actual knocking mechanism

def systematic_search(arena_width, arena_height, step_size):
    """
    Generate a systematic search path in a zigzag pattern to cover the entire arena.
    Args:
        arena_width (int): The width of the arena in units.
        arena_height (int): The height of the arena in units.
        step_size (int): The step size for each move in units.
    Returns:
        List of (x, y) positions defining the search path.
    """
    search_path = []  # List to store the generated search positions
    y = 0  # Start at the bottom of the arena
    direction = 1  # 1 means moving right, -1 means moving left

    while y < arena_height:
        if direction == 1:
            # Moving right across the arena
            for x in range(0, arena_width, step_size):
                search_path.append((x, y))
        else:
            # Moving left across the arena
            for x in range(arena_width - 1, -1, -step_size):
                search_path.append((x, y))

        # Move up to the next row
        y += step_size
        # Switch direction for zigzag pattern
        direction *= -1

    return search_path

def main():
    """
    Main function to coordinate the robot's box searching and knocking actions.
    Includes both box detection and systematic search.
    """
    # Define arena dimensions and step size for systematic search
    arena_width = 20  # Width of the arena in units
    arena_height = 10  # Height of the arena in units
    step_size = 2  # Step size for systematic search in units

    # Generate the systematic search path
    search_path = systematic_search(arena_width, arena_height, step_size)
    current_position_index = 0  # Keeps track of the robot's progress in the search path

    while True:
        # Step 1: Scan for boxes in the arena
        detected_boxes = scan_for_boxes()

        if detected_boxes:  # If any boxes are detected
            for box in detected_boxes:
                box_id = box["id"]  # Get the ID of the detected box
                position = box["position"]  # Get the position of the detected box

                # Check if the top box's ID does not belong to the robot
                if box_id not in robot_own_ids:
                    print(f"Found box with ID {box_id} at {position} that is not ours.")
                    # Move to the box's position and knock it over
                    move_to_position(position)
                    knock_over_stack(position)
                    break  # Exit the loop after knocking over one stack
        else:  # If no boxes are detected
            if current_position_index < len(search_path):  # If the search path is not complete
                # Get the next position in the search path
                next_position = search_path[current_position_index]
                print(f"No boxes detected. Moving to systematic search position {next_position}")
                # Move to the next position in the systematic search
                move_to_position(next_position)
                # Increment the position index to track progress
                current_position_index += 1
            else:  # If the search path is complete
                print("Systematic search complete. Restarting...")
                # Reset the search path to restart coverage of the arena
                current_position_index = 0

# Run the main function
main()

'''