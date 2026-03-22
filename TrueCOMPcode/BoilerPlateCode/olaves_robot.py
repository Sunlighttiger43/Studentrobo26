# olaves_robot.py
import math
from math import pi, sin, cos, atan2, sqrt
from time import time, struct_time
from typing import List, Tuple, Optional, Union
from TestCode.pid import PIDController
from zone_coords import zone_coords

import numpy as np
import sr.robot3 as sr

import logging
import json
import os

# Creating a logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Creating handlers
console = logging.StreamHandler()
console.setLevel(logging.DEBUG)
# file = logging.FileHandler(filename="robot.log")
# file.setLevel(logging.DEBUG)

# Adding handlers to logger
logger.addHandler(console)
# logger.addHandler(file)


class OlavesRobot(sr.Robot):
    def __init__(self):
        super().__init__(raw_ports=[("43439353536351815121", 115200)])
        print(self.arduinos.keys())
        logger.info("Initialising OlavesRobot")
        self.init_time: float = time()
        self._position: np.ndarray = [0, 0, 0]  # x, y, theta
        self._last_time_updated: float = 0  # seconds
        self.ticks = [0, 0]

        self.zone_coords = zone_coords[self.zone]

        self.pallet_dict = {
            0: list(i for i in range(100, 120)),
            1: list(i for i in range(120, 140)),
            2: list(i for i in range(140, 160)),
            3: list(i for i in range(160, 180)),
        }

        self.hirise_dict = {
            0: 195,
            1: 196,
            2: 197,
            3: 198,
        }

        # Might prove helpful to distinguish between physical robot and simulated robot
        # for testing
        self.is_physical_robot = True

        # Setting servo to default position for simulated robot:
        if not self.is_physical_robot:
            self.servo_board.servos[0].position = 1

        # We will load any external PID tunings here and initialise controllers for them
        # This allows for continuity between different testing runs
        self._initialise_PID_controllers()

        self.hardward_initialisation()

    def _load_pid_tunings_from_file(self) -> dict:
        """
        Loads PID tunings from a JSON file if it exists.
        Returns a dictionary of PID configs dictionaries keyed by name.
        """
        filename = "pid_tunings.json"
        if os.path.exists(filename):
            with open(filename, "r") as f:
                try:
                    data = json.load(f)
                    logger.info("[_load_pid_tunings_from_file] | Loaded PID tunings.")
                    return data
                except Exception as e:
                    logger.error(
                        f"[_load_pid_tunings_from_file] | Error loading PID config: {e}"
                    )
                    return {}
        else:
            logger.warning(
                "[_load_pid_tunings_from_file] No external PID config file found."
            )
            return {}

    def _initialise_PID_controllers(self) -> None:
        """
        Initialises all PID controllers at the class level.
        """
        pid_tunings = self._load_pid_tunings_from_file()

        # Retrieving PID Gains for isolated angle PID
        isolated_angle_pid_gains = pid_tunings.get(
            "isolated_angle_pid_gains",
            {"p": 0.4, "i": 0.2, "d": 0.3},  # fallback defaults
        )

        # Setting isolated angle PID
        self.isolated_angle_pid = PIDController(
            sp=0.0,
            p=isolated_angle_pid_gains["p"],
            i=isolated_angle_pid_gains["i"],
            d=isolated_angle_pid_gains["d"],
            integrator_min=-5.0,
            integrator_max=5.0,
        )

        # Retrieving PID Gains for move functions
        move_dist_pid_gains = pid_tunings.get(
            "move_pos_dist", {"p": 0.2, "i": 0.05, "d": 0.05}
        )
        move_angle_pid_gains = pid_tunings.get(
            "move_pos_angle", {"p": 0.4, "i": 0.1, "d": 0.3}
        )

        # Setting PIDs for move functions
        self.move_dist_pid = PIDController(
            sp=0.0,
            p=move_dist_pid_gains["p"],
            i=move_dist_pid_gains["i"],
            d=move_dist_pid_gains["d"],
            integrator_min=-5.0,
            integrator_max=5.0,
        )
        self.move_angle_pid = PIDController(
            sp=0.0,
            p=move_angle_pid_gains["p"],
            i=move_angle_pid_gains["i"],
            d=move_angle_pid_gains["d"],
            integrator_min=-5.0,
            integrator_max=5.0,
        )

    def get_comp_time(self) -> float:
        """
        Returns the time elapsed since the robot was initialised.

        Computes the difference between the current time and the initialisation time.

        Returns:
            float: Elapsed time in seconds.
        """
        elapsed_time = time() - self.init_time
        logger.debug(f"Computed elapsed time: {elapsed_time} seconds.")
        return elapsed_time

    def get_motor_ticks(self) -> Tuple[int, int]:
        """Returns the change in ticks since the last time it was checked.

        Calls the ruggeduino to return the total ticks in the left and right wheels. The
        ruggeduino attaches to interupts functions that increment a count for each tick of
        the encoder.

        Returns:
            Tuple[int, int]:
                A tuple of (delta_left_ticks, delta_right_ticks), representing
                the change in wheel encoder ticks since this method was last called.
        """
        logger.debug("Requesting motor ticks from ruggeduino.")
        tickL = int(self.arduino.command(">"))
        tickR = int(self.arduino.command("<"))
        logger.debug(f"Raw tick counts from Arduino: left={tickL}, right={tickR}")

        last = self.ticks
        self.ticks = (tickL, tickR)
        delta = (self.ticks[0] - last[0], self.ticks[1] - last[1])
        logger.debug(f"Delta ticks computed: left={delta[0]}, right={delta[1]}")

        return delta

    def _update_time(self) -> struct_time:
        """
        Updates and returns the internal timestamp.

        This method records the current competition time (in seconds since the
        robot started) to `self._last_time_updated` and logs the new value.

        Returns:
            float: The updated competition time in seconds.
        """
        self._last_time_updated = self.get_comp_time()
        logger.debug(f"Updated time: {self._last_time_updated:.3f}s")
        return self._last_time_updated

    def _encoder_pos(
        self,
    ) -> np.ndarray:
        """
        Calculates the robot's position and orientation based on encoder ticks.

        Uses the difference in wheel rotations (based on encoder ticks) to update
        and return the robot's estimated position as an array [x, y, theta].

        Returns:
            np.ndarray: A 3-element numpy array containing the updated
                        [x, y, theta] of the robot.
        """
        logger.info("[_encoder_pos] | Calculating location using encoders")
        # MATHS FROM: https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf

        logger.info("Calculating position from encoder ticks.")

        TICKS_PER_ROT = 2880
        WHEEL_RADIUS = 0.072
        ROBOT_WIDTH = 0.42
        HALF_ROBOT_WIDTH = ROBOT_WIDTH / 2

        # Get the wheel ticks since last check
        ticks = self.get_motor_ticks()
        logger.debug(f"[_encoder_pos] | Encoder ticks received: {ticks}.")

        # Update the timestamp to compute velocity based on delta time
        previous_time = self._last_time_updated
        delta_t = self._update_time() - previous_time

        r_x, r_y, r_theta = self._position

        # Convert ticks to wheel rotation (in rotations) then to radians
        delta_phi = tuple(tick / TICKS_PER_ROT for tick in ticks)

        # Convert rotations to linear velocities (v_l, v_r)
        v_l, v_r = (phi * 2 * pi * WHEEL_RADIUS / delta_t for phi in delta_phi)

        # Compute the Instantaneous Center of Curvature (ICC)
        if (v_r - v_l) != 0:
            ICC_r = HALF_ROBOT_WIDTH * (v_l + v_r) / (v_r - v_l)
        else:
            ICC_r = HALF_ROBOT_WIDTH * (v_l + v_r) / 1e-10  # Avoid division by zero

        ICC_omega = (v_r - v_l) / ROBOT_WIDTH
        ICC = np.array([r_x - ICC_r * sin(r_theta), r_y + ICC_r * cos(r_theta), 0])

        # Rotation matrix approach
        rotation_matrix = np.array(
            [
                [cos(ICC_omega * delta_t), -sin(ICC_omega * delta_t), 0],
                [sin(ICC_omega * delta_t), cos(ICC_omega * delta_t), 0],
                [0, 0, 1],
            ]
        )

        pos = (rotation_matrix @ (self._position - ICC)) + np.array(
            [ICC[0], ICC[1], ICC_omega * delta_t]
        )
        self._position = pos

        logger.debug(
            f"[_encoder_pos] | Encoder position updated to [x={pos[0]:.3f}, y={pos[1]:.3f}, theta={pos[2]:.3f}] after {delta_t:.3f}s."
        )

        return pos

    def sleep(self, duration: float) -> None:
        """
        Pauses execution for a specified duration.

        Acts as time.sleep, but tailored for the simulation environment where robot.sleep() may not function as expected.

        Parameters:
            duration (float): The amount of time to sleep in seconds.
        """
        logger.info(f"[sleep] | Sleeping for {duration} seconds.")
        super().sleep(duration)

    def _turn(self, angle: float) -> None:
        """
        Performs a dead reckoning turn.

        This method should not be used outside the OlavesRobot subclass. It calculates the duration required to turn the specified angle
        based on a predefined multiplier and adjusts motor powers accordingly.

        Parameters:
            angle (float): The angle to turn in radians. Positive values indicate a left turn, while negative values indicate a right turn.
        """
        ANGLE_TO_TIME = 14.2  # Multiplier to make the turn more accurate
        direction = "left" if angle > 0 else "right"
        logger.info(f"[_turn] | Turning {direction} by {abs(angle)} radians.")

        if angle > 0:
            # Turn left
            self.power((0.4, -0.4), angle / pi * ANGLE_TO_TIME)
        else:
            # Turn right
            self.power((-0.4, 0.4), abs(angle) / pi * ANGLE_TO_TIME)
        logger.debug(f"[_turn] | Completed turn {direction}.")

    def _move(self, distance: float) -> None:
        """
        Performs a dead reckoning move.

        This method should not be used outside the OlavesRobot subclass. It calculates the duration required to move the specified distance
        based on a predefined multiplier and adjusts motor powers accordingly.

        Parameters:
            distance (float): The distance to move in meters. Positive values move the robot forward, while negative values move it backward.
        """
        DIST_TO_TIME = 5
        direction = "forward" if distance > 0 else "backward"

        if distance > 0:
            self.power((0.4, 0.4), distance * DIST_TO_TIME)
        else:
            self.power((-0.4, -0.4), abs(distance) * DIST_TO_TIME)

    def _smart_turn(self, angle: float) -> None:
        """
        Uses PIDControllers to turn a specified 'angle' (float).

        Parameters:
            angle (float): The angle in radians the robot is to turn
        """
        r_x, r_y, r_theta = self.get_pos()

        target_orientation = r_theta + angle

        self.set_robot_orientation(target_orientation=target_orientation)

    def _smart_move(self, distance: float) -> None:
        """
        Uses PIDControllers to move the robot a specified distance (float) forward.

        Can be thought of as adding a vector to the robot position.

        Parameters:
            distance (float): The distance in meters the robot is to move forward
        """
        r_x, r_y, r_theta = self.get_pos()

        destination_x = distance * cos(r_theta) + r_x
        destination_y = distance * sin(r_theta) + r_y

        self.move_pos((destination_x, destination_y))

    def get_marker_by_id(
        self, marker_id: int, search_angle: float = pi * 2, speed: float = 0.2
    ) -> Optional[sr.marker]:
        """
        Retrieves a marker object by its ID.

        Searches the robot's camera feed for a marker with the specified ID.

        Parameters:
            marker_id (int): The ID of the marker to search for.
            search_angle (float): The maximum angle (in radians) to rotate while searching.
                                Defaults to 2pi (a full rotation).
            speed (float): The speed at which to rotate the robot.

        Returns:
            The marker object if found, or None if the marker could not be located.
        """
        logger.info(
            f"[get_marker_by_id] | Retrieving marker object for marker with id {marker_id}"
        )
        # Check if the marker is already in view.
        markers = [m for m in self.camera.see() if m.id == marker_id]
        if markers:
            marker = sorted(markers, key=lambda x: x.position.distance)[0]
            logger.info(
                f"[get_marker_by_id] Marker {marker_id} found in initial check."
            )
            return marker

        logger.info(
            f"[get_marker_by_id] Marker {marker_id} not visible initially. Initiating search..."
        )

        # turned_angle = 0.0
        # # Rotate in increments until we've turned the maximum allowed angle.
        # while turned_angle < search_angle:
        #     # Rotate the robot slightly; adjust the time (0.5 seconds here) as needed.
        #     self._turn(pi / 16)
        #     turned_angle += pi / 16

        #     # Check the camera feed after the turn.
        #     markers = [m for m in self.camera.see() if m.id == marker_id]
        #     if markers:
        #         marker = sorted(markers, key=lambda x: x.position.distance)[0]
        #         logger.info(
        #             f"[get_marker_by_id] Marker {marker_id} found after turning {turned_angle:.2f} radians."
        #         )
        #         self.power((0.0, 0.0), 0.3)  # Stop the robot.
        #         return marker

        # If the search completes without finding the marker, stop the robot.
        self.power((0.0, 0.0), 0.0)
        logger.info(
            f"[get_marker_by_id] Could not find marker {marker_id} after rotating {search_angle:.2f} radians."
        )
        return None

    def get_marker_by_idrange(
        self, marker_id_list: list, search_angle: float = pi * 2, speed: float = 0.2
    ) -> Optional[sr.marker]:
        """
        Retrieves a marker object by its ID.

        Searches the robot's camera feed for a marker with the specified ID.

        Parameters:
            marker_id (int): The ID of the marker to search for.
            search_angle (float): The maximum angle (in radians) to rotate while searching.
                                Defaults to 2pi (a full rotation).
            speed (float): The speed at which to rotate the robot.

        Returns:
            The marker object if found, or None if the marker could not be located.
        """
        # Check if the marker is already in view.
        markers = [m for m in self.camera.see() if m.id in marker_id_list]
        if markers:
            marker = sorted(markers, key=lambda x: x.position.distance)[0]
            return marker

        turned_angle = 0.0
        # Rotate in increments until we've turned the maximum allowed angle.
        while turned_angle < search_angle:
            # Rotate the robot slightly; adjust the time (0.5 seconds here) as needed.
            self._turn(pi / 16)
            turned_angle += pi / 16

            # Check the camera feed after the turn.
            markers = [m for m in self.camera.see() if m.id in marker_id_list]
            if markers:
                marker = sorted(markers, key=lambda x: x.position.distance)[0]
                self.power((0.0, 0.0), 0.0)  # Stop the robot.
                return marker

        # If the search completes without finding the marker, stop the robot.
        self.power((0.0, 0.0), 0.0)
        return None

    def _turn_to_marker(self, marker_id: int) -> None:
        """
        Re-orients the robot to face towards a specified marker.

        **Note:** This method is not required for the virtual league and remains unimplemented.

        Assumes that the robot is positioned directly in front of the marker.

        Parameters:
            marker_id (int): The unique identifier of the target marker.

        Raises:
            NotImplementedError: This method is intentionally unimplemented for the virtual league.
        """
        logger.info(f"[_turn_to_marker] | Turning to marker with id {marker_id}")

        ANGLE_TOLERANCE = 0.05
        ANGLE_TO_TIME = 0.1

        # Use the get_marker_by_id function to get the initial marker object.
        marker = self.get_marker_by_id(marker_id)
        if not marker:
            logger.error(
                f"[_turn_to_marker] Marker {marker_id} not found. Aborting turn."
            )
            return

        init_angle = marker.position.horizontal_angle
        angle_error = init_angle

        logger.debug(f"[_turn_to_marker] | Initial Angle Error: {angle_error:.3f} rad")

        iterations = 0
        consecutive_lost_frames = 0
        max_lost_frames = 5

        while abs(angle_error) > ANGLE_TOLERANCE and iterations < 100:
            # Calculate PID corrections
            angle_correction = self.isolated_angle_pid.getCorrection(angle_error)

            logger.debug(
                f"[_turn_to_marker] | Angle Correction: {angle_correction:.3f}"
            )

            # Convert corrections into left/right motor power
            left_power = -angle_correction
            right_power = +angle_correction

            # Apply scaling with minimum power
            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.1, center=0
            )

            logger.debug(
                f"[_turn_to_marker] | Left Power: {left_power:.3f}, Right Power: {right_power:.3f}"
            )

            # Determine dynamic time_step based on distance error
            # This can help in smoother control as the robot approaches the target
            time_step = min(
                ANGLE_TO_TIME,
                max((init_angle - angle_error) * ANGLE_TO_TIME, ANGLE_TO_TIME * 0.25),
            )

            # Apply motor power
            self.power((left_power, right_power), time_step, off=False)

            # Update marker information
            marker = self.get_marker_by_id(marker_id)
            if not marker:
                logger.warning(
                    f"[_turn_to_marker] | Lost marker {marker_id}. Conducting search."
                )
                consecutive_lost_frames += 1
                if consecutive_lost_frames >= max_lost_frames:
                    logger.error(
                        "[_turn_to_marker] | Marker lost too many times, aborting."
                    )
                    break
                continue
            else:
                consecutive_lost_frames = 0
                angle_error = marker.position.horizontal_angle
                logger.debug(
                    f"[_turn_to_marker] Updated Angle Error: {angle_error:.3f} rad"
                )

            angle_error = marker.position.horizontal_angle

            logger.debug(
                f"[_turn_to_marker] | Updated Angle Error: {angle_error:.3f} rad"
            )

            iterations += 1

        # Stop the robot once target is reached
        self.power((0.0, 0.0), 0.0)
        logger.debug("[_turn_to_marker] | Completed or aborted turning to marker.")

    def _align_with_marker(self, marker, target_marker_pos: np.ndarray) -> None:
        """
        Aligns the robot with the specified marker.

        This method is not needed for the virtual league and is intentionally left unimplemented.

        Parameters:
            marker (sr.Marker): The marker object to align with.
            target_marker_pos (np.ndarray): The position of the target marker.

        Raises:
            NotImplementedError: Always raised as this method is not required for the virtual league.
        """
        target_position = (
            target_marker_pos[0] * cos(target_marker_pos[2]),
            target_marker_pos[1] * sin(target_marker_pos[2]),
        )

        self.move_pos(target_position=target_position)

        self.set_robot_orientation(
            target_orientation=self._restrict_angle_range(target_marker_pos[2] + pi)
        )

    def get_marker_pos(self, marker_id: int) -> Tuple[float, float]:
        # NEEDS TO BE TESTED - IMPLEMENTED BY AARIT
        """
        Calculates the absolute position of a marker based on its relative position to the robot.

        Parameters:
            marker_id (int): The ID of the marker.

        Returns:
            Tuple[float, float, float]: A tuple containing the (x, y, theta) coordinates and orientation of the marker.

        Raises:
            ValueError: If the marker with the specified ID is not found.
        """
        logger.info(
            f"[get_marker_pos] | Calculating position for marker ID {marker_id}."
        )

        marker = self.get_marker_by_id(marker_id=marker_id)
        if not marker:
            logger.error(
                f"[get_marker_pos] Marker {marker_id} not found. Aborting localisation."
            )
            return

        marker_orientation = marker.orientation.yaw
        marker_distance = marker.position.distance
        marker_horizontal_angle = marker.position.horizontal_angle

        logger.debug(
            f"[get_marker_pos] | Target marker yaw: {marker_orientation}, Target marker distance: {marker_distance}"
        )

        # Retrieves the robot current position to estimate where the marker is relative to it
        robot_current_pos = (
            self.get_pos()
        )  # Here I am assuming that the angle is a bearing in radians
        robot_x, robot_y, robot_theta = (
            robot_current_pos[0],
            robot_current_pos[1],
            robot_current_pos[2],
        )
        # Idea here is to utilise the cosine rule by making an isosceles triangle with the 'A' angle being marker orientation
        x_vector = (
            sqrt(
                2 * marker_distance**2 - (marker_distance**2) * cos(marker_orientation)
            )
            / 2
        )
        y_vector = sqrt((marker_distance**2) - (x_vector**2))

        logger.debug(f"[get_marker_pos] | x_vector: {x_vector}, y_vector: {y_vector}")

        # Checks whether the actual marker is behind the robot or not, by looking at the orientation of the robot and check what the orientation of the marker is to add either negative x vector or positive x vector
        if (robot_theta > (1.5) * pi and robot_theta < 2 * pi) or (
            robot_theta > 0 and robot_theta < 0.5 * pi
        ):
            if marker_orientation > 0:
                x_coord = robot_x - x_vector
                y_coord = y_vector + robot_y
            else:
                x_coord = x_vector + robot_x
                y_coord = y_vector + robot_y
        else:
            if marker_orientation > 0:
                x_coord = robot_x + x_vector
                y_coord = y_vector - robot_y
            else:
                x_coord = x_vector - robot_x
                y_coord = y_vector - robot_y

        m_theta = pi + robot_theta + marker_orientation - marker_horizontal_angle

        logger.debug(
            f"[get_marker_pos] | Marker coordinates: {(x_coord, y_coord)}, Marker orientation: {m_theta}"
        )
        return (x_coord, y_coord, m_theta)

    def get_pos(
        self, test_markers: Optional[List[sr.marker]] = None
    ) -> Optional[Tuple[float, float, float]]:
        """
        Estimates the robot's position and orientation.

        If `test_markers` are provided, these marker observations are used instead of the camera input, facilitating testing.

        Parameters:
            test_markers (Optional[List[sr.Marker]]): A list of marker observations to use for position estimation. Defaults to None.

        Returns:
            Optional[Tuple[float, float, float]]: A tuple containing the (x, y) position and orientation (theta) of the robot.
                                                 Returns None if insufficient markers are visible.

        Raises:
            NotImplementedError: Indicates that the method's implementation is pending.
        """
        # logger.warning("get_pos method needs to be implemented.")
        # # TODO: Implement position estimation logic here
        raise NotImplementedError("get_pos method needs to be implemented.")

        return self._encoder_pos()

    def set_robot_orientation(self, target_orientation: float) -> None:
        """
        Sets the robot's orientation to a specified angle using PID control.

        The method adjusts the robot's orientation from the current angle `r_theta` to the target angle `target_orientation`, minimising the error through PID control.

        Parameters:
            target_orientation (float): The target orientation angle in radians.
        """
        logger.info(
            f"[set_robot_orientation] | Setting robot orientation to {target_orientation}"
        )

        ANGLE_TOLERANCE = 0.05
        ANGLE_TO_TIME = 0.1

        r_x, r_y, r_theta = self._encoder_pos()

        init_angle = target_orientation - r_theta
        angle_error = init_angle

        logger.debug(
            f"[set_robot_orientation] | Initial Angle Error: {init_angle:.3f} rad"
        )

        iterations = 0

        while abs(angle_error) > ANGLE_TOLERANCE and iterations < 100:
            # Calculate PID corrections
            angle_correction = self.isolated_angle_pid.getCorrection(angle_error)

            logger.debug(
                f"[set_robot_orientation] | Angle Correction: {angle_correction:.3f}"
            )

            # Convert corrections into left/right motor power
            left_power = -angle_correction
            right_power = +angle_correction

            # Apply scaling with minimum power
            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.1, center=0
            )

            logger.debug(
                f"[set_robot_orientation] | Left Power: {left_power:.3f}, Right Power: {right_power:.3f}"
            )

            # Determine dynamic time_step based on distance error
            # This can help in smoother control as the robot approaches the target
            time_step = min(
                ANGLE_TO_TIME,
                max((init_angle - angle_error) * ANGLE_TO_TIME, ANGLE_TO_TIME * 0.25),
            )

            # Apply motor power
            self.power((left_power, right_power), time_step, off=False)

            r_x, r_y, r_theta = self.get_pos()

            angle_error = target_orientation - r_theta

            logger.debug(
                f"[set_robot_orientation] | Updated Angle Error: {angle_error:.3f} rad"
            )

            iterations += 1

        # Stop the robot once target is reached
        self.power((0.0, 0.0), 0.0)
        logger.debug(
            "[set_robot_orientation] | Completed or aborted setting orientation."
        )

    @staticmethod
    def _restrict_angle_range(theta: float) -> float:
        """
        Sets the input angle between -pi and pi

        Parameters:
            theta (float): Input angle to be restricted

        Returns:
            phi (float): The angle set to the restricted domain
        """
        if theta <= -pi:
            phi = theta + 2 * pi
        elif theta > pi:
            phi = theta - 2 * pi
        else:
            phi = theta

        return phi

    def power(
        self,
        wheels: Tuple[float, float],
        duration: float,
        off: bool = True,
    ) -> None:
        """
        Controls the motors by setting their power levels.

        Adjusts the power of the left and right motors for a specified duration. Optionally turns off the motors after the duration.

        Parameters:
            wheels (Tuple[float, float]):
                - First element controls the left wheel's power.
                - Second element controls the right wheel's power.
                Positive values move the wheels forward, while negative values move them backward.
            duration (float): The duration in seconds for which the motors should run.
            off (bool, optional): Whether to turn off the motors after the duration. Defaults to True.

        Raises:
            ValueError: If wheel power values are outside the allowable range after correction.
        """
        corrected_wheels = self._wonky_wheels_correction(wheels)
        logger.debug(
            f"Original wheel powers: {wheels}. Corrected wheel powers: {corrected_wheels}."
        )
        self._prev_power = corrected_wheels
        self.motor_board.motors[1].power = self._clamp(corrected_wheels[1])
        self.motor_board.motors[0].power = self._clamp(corrected_wheels[0])
        logger.debug(
            f"Setting motor powers to {self._prev_power} for {duration} seconds."
        )
        self.sleep(duration)
        if off:
            self.motor_board.motors[0].power = 0.0
            self.motor_board.motors[1].power = 0.0
            logger.info("Motors turned off.")

    @staticmethod
    def _clamp(value: float, min_value: float = -1.0, max_value: float = 1.0) -> float:
        """
        Clamps a value to the specified range.

        Ensures that the provided value does not exceed the defined minimum and maximum boundaries.

        Parameters:
            value (float): The value to clamp.
            min_value (float, optional): The minimum allowable value. Defaults to -1.0.
            max_value (float, optional): The maximum allowable value. Defaults to 1.0.

        Returns:
            float: The clamped value.
        """
        clamped_value = max(min_value, min(max_value, value))
        logger.debug(f"[_clamp] | Clamping value {value} to {clamped_value}.")
        return clamped_value

    def _scale(
        self,
        left: float,
        right: float,
        max_std: float = 3,
        center: float = 2.0,
        min_power: float = 0.10,
    ) -> tuple[float, float]:
        """
        Scales the two motor power values to ensure balanced and stable movement.

        The scaling process adjusts the power values to meet the following criteria:
        1. The population standard deviation between the two values does not exceed `max_std`.
        2. The mean of the two values is shifted to the specified `center` value.
        3. The final values are clamped within the range [-1.0, 1.0] to adhere to motor power limits.
        4. Applies a minimum power threshold (`min_power`) to overcome static friction and reduce jitter.

        Parameters:
            left (float): Desired power level for the left motor. Range: [-1.0, 1.0].
            right (float): Desired power level for the right motor. Range: [-1.0, 1.0].
            max_std (float, optional): Maximum allowed standard deviation between motor powers. Defaults to 3.0.
            center (float, optional): Target mean value for the motor powers after scaling. Defaults to 0.45.
            min_power (float, optional): Minimum non-zero power to apply to each motor to ensure movement. Defaults to 0.15.

        Returns:
            Tuple[float, float]: The scaled (left, right) motor power values, each within [-1.0, 1.0].

        Raises:
            ValueError: If `min_power` is negative or exceeds motor power limits.
        """
        logger.info(
            f"[_scale] | Scaling motor powers around {center}, with a max spread of {max_std}. Left Power: {left}, Right Power: {right}"
        )
        if min_power < 0 or min_power > 1.0:
            logger.error(
                f"Invalid min_power value: {min_power}. Must be between 0 and 1."
            )
            raise ValueError("min_power must be between 0 and 1.")

        # # # Compute mean
        # # mu = (left + right) / 2.0

        # # # Compute population standard deviation for two points
        # # sd = sqrt(((left - mu) ** 2 + (right - mu) ** 2) / 2.0)

        # # # Scale down if std exceeds max_std
        # # if sd > max_std and sd > 1e-9:
        # #     scale_factor = max_std / sd
        # #     left = mu + scale_factor * (left - mu)
        # #     right = mu + scale_factor * (right - mu)

        # # # Shift mean to center
        # # new_mu = (left + right) / 2.0
        # # shift = center - new_mu
        # # left += shift
        # # right += shift

        # Apply minimum power threshold
        left, right = self._apply_min_power(left, right, min_power)

        # Clamp to [-1.0, 1.0]
        left = self._clamp(left, -1.0, 1.0)
        right = self._clamp(right, -1.0, 1.0)

        return (left, right)

    def _apply_min_power(
        self, left: float, right: float, min_power: float = 0.15
    ) -> tuple[float, float]:
        """
        Applies a minimum power threshold to both left and right motor powers.

        Parameters:
            left (float): Left motor power.
            right (float): Right motor power.
            min_power (float, optional): Minimum power to apply. Defaults to 0.15.

        Returns:
            (float, float): Adjusted (left, right) motor powers.
        """
        logger.debug(
            f"[_apply_min_power] Checking power is greater than {min_power}. Left Power: {left}, Right Power: {right}"
        )

        def apply_min(val):
            # If turning on the spot (opposite signs), don't forcibly push them in same direction
            if 0.0 < val < min_power:
                return min_power
            elif -min_power < val < 0.0:
                return -min_power
            return val

        # If signs of left and right differ, it's likely a turn.
        # We'll apply a smaller threshold so it can spin in place:
        if left * right < 0:
            # turning case
            turning_min = min_power
            left_adj = (
                -turning_min
                if (left < 0 and left > turning_min)
                else (turning_min if (left > 0 and left < turning_min) else left)
            )
            right_adj = (
                -turning_min
                if (right < 0 and right > turning_min)
                else (turning_min if (right > 0 and right < turning_min) else right)
            )

            return (left_adj, right_adj)
        else:
            # normal forward/back
            left = apply_min(left)
            right = apply_min(right)
            return (left, right)

    def _wonky_wheels_correction(
        self, wheels: Tuple[float, float]
    ) -> Tuple[float, float]:
        """
        Applies correction to the wheel power values.

        This method can be customised to correct any discrepancies in wheel behavior. Currently reserved for physical robot implementations.

        For the virtual league, no correction is applied.

        Parameters:
            wheels (Tuple[float, float]): The raw wheel power values as (left, right).

        Returns:
            Tuple[float, float]: The corrected wheel power values.

        Raises:
            NotImplementedError: Indicates that wheel correction is not implemented for the virtual league.
        """
        logger.warning(
            "_wonky_wheels_correction is not implemented for the virtual league."
        )
        return wheels  # Assuming no correction needed in simulation

    def zone_pallets(self) -> List[sr.marker]:
        return self.pallet_dict[self.zone]

    def zone_hirise(self):
        return self.hirise_dict[self.zone]

    def zone_base(self):
        return 7 * (self.zone)

    def zone_outer(self):
        return 195 + self.zone

    def get_marker_type(self, marker: Union[sr.marker, int]) -> Optional[str]:
        """
        Determines the type of a marker based on its ID.

        Categorises markers into predefined types such as walls, pallets, and highrise markers based on their ID ranges.

        Parameters:
            marker (Union[sr.Marker, int]): The marker object or raw marker ID integer.

        Returns:
            Optional[str]: The type of marker, which can be one of the following:
                - 'wall' (IDs 0-27)
                - 'pallet0' (IDs 100-119)
                - 'pallet1' (IDs 120-139)
                - 'pallet2' (IDs 140-159)
                - 'pallet3' (IDs 160-179)
                - 'highrise' (IDs 195-199)
                - None if the marker ID does not match any known type.

        Raises:
            ValueError: If the provided marker is neither an sr.Marker instance nor an integer.
        """
        logger.info(
            f"[get_marker_type] | Getting the marker type of marker {marker.id}"
        )
        if isinstance(marker, sr.marker):
            marker_id = marker.id
        elif isinstance(marker, int):
            marker_id = marker
        else:
            logger.error(f"[get_marker_type] | Invalid marker type: {type(marker)}")
            raise ValueError(f"[get_marker_type] | Invalid marker type: {type(marker)}")

        logger.debug(f"[get_marker_type] | Determining type for marker ID: {marker_id}")

        match marker_id:
            case m_id if 0 <= m_id <= 27:
                return "wall"
            case m_id if 100 <= m_id <= 119:
                return "pallet0"
            case m_id if 120 <= m_id <= 139:
                return "pallet1"
            case m_id if 140 <= m_id <= 159:
                return "pallet2"
            case m_id if 160 <= m_id <= 179:
                return "pallet3"
            case m_id if 195 <= m_id <= 199:
                return "highrise"
            case _:
                logger.warning(f"[get_marker_type] | Unknown marker ID: {marker_id}")
                return None

    def detect_highrise_pallets(self):
        """
        Determines the amount of high rise pallets the robot can see for our team
        We do this by checking if the pitch angle and if it is tilted up (ie less than 0 radians)
        Parameters:
            None
        Output:
            list output [numberofhighrises, onfloormarkers]
        """

        logger.info(
            f"[detect_highrise_pallets] | Determining the number of high rises from our team"
        )

        markers = self.camera.see()
        highrisers = 0
        for marker in markers:
            if marker.orientation.pitch < 0 and (
                self.get_marker_type(marker) == "pallet0"
                or self.get_marker_type(marker) == "pallet1"
                or self.get_marker_type(marker) == "pallet2"
                or self.get_marker_type(marker) == "pallet3"
            ):
                highrisers = highrisers + 1

        return [highrisers, 6 - highrisers]

    def move_to_marker(self, marker_id: int) -> None:
        """
        Moves the robot to a specified marker using PID controllers.

        When finishes, assumes that the marker has already been found

        Parameters:
            marker_id (int): The ID of the target marker.

        Raises:
            "MarkerNotFound" : Exception when the specified marker is not found by the robot
        """
        logger.info(f"[move_to_marker] | Moving to Marker: {marker_id}")

        DIST_TOLERANCE = 0.1  # meters
        ANGLE_TOLERANCE = 0.05  # radians
        DIST_TO_TIME = 0.05  # Base duration for movement

        marker = self.get_marker_by_id(marker_id=marker_id)
        if not marker:
            logger.error(
                f"[move_to_marker] Marker {marker_id} not found. Aborting turn."
            )
            # raise Exception("MarkerNotFound")
            return

        init_dist = marker.position.distance / 1000.0  # Convert mm to meters

        dist_error = init_dist
        angle_error = marker.position.horizontal_angle

        logger.debug(
            f"[move_to_marker] | Initial Distance Error: {dist_error:.3f} m, Initial Angle Error: {angle_error:.3f} rad"
        )

        iterations = 0
        consecutive_lost_frames = 0
        max_lost_frames = 5

        while (
            (dist_error > DIST_TOLERANCE or abs(angle_error) > ANGLE_TOLERANCE)
            and iterations < 1000
            and consecutive_lost_frames < max_lost_frames
        ):
            # Calculate PID corrections
            dist_correction = -self.move_dist_pid.getCorrection(dist_error)
            angle_correction = self.move_angle_pid.getCorrection(angle_error)

            logger.debug(
                f"[move_to_marker] | Distance Correction: {dist_correction:.3f}, Angle Correction: {angle_correction:.3f}"
            )

            # Convert corrections into left/right motor power
            left_power = dist_correction - angle_correction
            right_power = dist_correction + angle_correction

            # Apply scaling with minimum power
            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.15
            )

            logger.debug(
                f"[move_to_marker] | Left Power: {left_power:.3f}, Right Power: {right_power:.3f}"
            )

            # Determine dynamic time_step based on distance error
            # This can help in smoother control as the robot approaches the target
            time_step = min(
                DIST_TO_TIME,
                max((init_dist - dist_error) * DIST_TO_TIME, DIST_TO_TIME * 0.1),
            )

            # Apply motor power
            self.power((left_power, right_power), time_step, off=False)

            # Update marker information
            marker = self.get_marker_by_id(marker_id)
            if not marker:
                if consecutive_lost_frames >= max_lost_frames:
                    logger.error(
                        "[move_to_marker] | Marker lost too many times, aborting."
                    )
                    break
                if dist_error <= 3 * DIST_TOLERANCE:
                    # Assume the marker is most likely collected already
                    break
                logger.warning(
                    f"[move_to_marker] | Lost marker {marker_id}. Conducting search."
                )
                consecutive_lost_frames += 1
                continue
            else:
                consecutive_lost_frames = 0
                angle_error = marker.position.horizontal_angle
                dist_error = marker.position.distance / 1000.0
                logger.debug(
                    f"[move_to_marker] Updated Distance Error: {dist_error:.3f} m, Updated Angle Error: {angle_error} rads."
                )

            iterations += 1

        self.power((0.0, 0.0), 0.0)

    def _move_to_marker_finalisation(self) -> None:
        """
        Finalises the motion for move_to_marker, llowing the robot to close the distance without the use of the camera to determine distance.

        Uses ultrasound sensor to determine distance from marker
        """
        logger.info(
            f"[_move_to_marker_finalisation] | Minimising distance between robot and marker in the absence of camera feedback"
        )
        DIST_TOLERANCE = 0.01  # meters

        init_dist = (
            self.arduino.ultrasound_measure(2, 3) / 1000.0
        )  # Convert mm to meters

        dist_error = init_dist

        logger.debug(
            f"[_move_to_marker_finalisation] | Initial Distance Error: {dist_error:.3f} m"
        )

        # TODO: Add error exception handling here to check that the ultrasound reading
        # isn't invalid (e.g. reading of 2m is obviously not a marker that can be reached
        # reasonably using ultrasound)

        iterations = 0
        while dist_error > DIST_TOLERANCE and iterations != 100:
            logger.debug(
                f"[_move_to_marker_finalisation] | Moving forward {dist_error}m"
            )

            self._smart_move(dist_error * 0.7)

            dist_error = self.arduino.ultrasound_measure(2, 3) / 1000.0
            iterations += 1

        logger.debug(
            "[_move_to_marker_finalisation] | Marker in position. Exiting Movement"
        )

    def move_pos(self, target_position: Tuple[float, float]) -> None:
        """
        Moves the robot to a specific (x, y) position using PID controllers for angle and distance.

        The method calculates the required adjustments in motor power to navigate the robot to the desired position accurately.

        Parameters:
            target_position (Tuple[float, float]): The (x, y) coordinates to move the robot to.

        Raises:
            NotImplementedError: Indicates that the method's implementation is pending.
        """
        logger.info(f"[move_pos] | Moving to position {target_position}.")

        DIST_TOLERANCE = 0.2  # meters
        ANGLE_TOLERANCE = 0.1  # radians
        DIST_TO_TIME = 0.5  # Base duration for movement

        current_position = self.get_pos()

        current_coords, phi = current_position[:2], current_position[2]

        destination_vector = target_position - current_coords
        angle = atan2(destination_vector[1], destination_vector[0])
        distance = np.linalg.norm(destination_vector)

        init_dist = distance  # Convert mm to meters
        dist_error = init_dist

        # Normalising angle error
        angle_error = angle - phi
        angle_error = (angle_error + pi) % (2 * pi) - pi

        logger.debug(
            f"[move_pos] | Initial Distance Error: {dist_error:.3f} m, Initial Angle Error: {angle_error:.3f} rad"
        )

        iterations = 0

        while (
            dist_error > DIST_TOLERANCE or abs(angle_error) > ANGLE_TOLERANCE
        ) and iterations != 1000:
            # Calculate PID corrections
            dist_correction = -self.move_dist_pid.getCorrection(dist_error)
            angle_correction = self.move_angle_pid.getCorrection(angle_error)

            logger.debug(
                f"[move_pos] | Distance Correction: {dist_correction:.3f}, Angle Correction: {angle_correction:.3f}"
            )

            # Convert corrections into left/right motor power
            left_power = dist_correction - angle_correction
            right_power = dist_correction + angle_correction

            # Apply scaling with minimum power
            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.15
            )

            logger.debug(
                f"[move_pos] | Left Power: {left_power:.3f}, Right Power: {right_power:.3f}"
            )

            # Determine dynamic time_step based on distance error
            # This can help in smoother control as the robot approaches the target
            time_step = min(
                max((dist_error / init_dist) * DIST_TO_TIME, 0.5 * DIST_TO_TIME),
                DIST_TO_TIME,
            )

            # Apply motor power
            self.power((left_power, right_power), time_step, off=False)

            current_position = self.get_pos()

            current_coords, phi = current_position[:2], current_position[2]

            destination_vector = np.array(target_position) - current_coords
            dist_error = np.linalg.norm(destination_vector)
            # Recompute angle error
            desired_angle = atan2(destination_vector[1], destination_vector[0])
            angle_error = desired_angle - phi
            angle_error = (angle_error + pi) % (2 * pi) - pi

            logger.debug(
                f"[move_pos] | Updated Distance Error: {dist_error:.3f} m, Updated Angle Error: {angle_error:.3f} rad"
            )

            iterations += 1

        # Stop the robot once target is reached
        self.power((0.0, 0.0), 0.0)
        logger.info("Position Reached")

    def camera_pos(self) -> Optional[Tuple[float, float]]:
        """
        Estimate position from camera alone, if we see known field markers with known global positions.

        DOES NOT WORK YET, DON'T USE

        Returns:
            (x, y) or None if we cannot see any known marker or have no knowledge of marker positions.
        """
        logger.info(f"[camera_pos] | Using camera to find location")
        markers = self.camera.see()
        # Dictionary of known markers to positions:
        zone_coordinates = self.zone_coords

        visible_known_markers = [m for m in markers if m.id in range(27)]
        if not visible_known_markers:
            return self._encoder_pos()
        if len(visible_known_markers) < 2:
            return self._encoder_pos()

        # This method uses two of the markers and uses a predetermined formula with proof already figured out
        marker = sorted(visible_known_markers, key=lambda x: x.position.distance)[0]
        # Let a = first marker x position, b = first marker y position, c = second marker x position, d = second marker y position
        a, b, c, d = marker[0][1], marker[0][2], marker[1][1], marker[1][2]
        # Let theta and alpha also be angles for first marker and second marker respectively - need to check if MARKER ID IS THIRD POSITION
        theta, alpha = marker.orientation.yaw(marker[0][0]), marker.orientation.yaw(
            marker[1][0]
        )

        x = ((math.tan(theta) * a) + b - a + (math.tan(alpha) * c)) / (
            (math.tan(theta)) + (math.tan(alpha))
        )
        y = math.tan(theta) * x - math.tan(theta) * a + b
        # Coordinates returned back - this was proved GEOMETRICALLY, See AARIT for Proof
        return (x, y)

    def pick_hold_box(self):
        """
        Picks up and holds the box.

        Assumes that the box is directly ahead of the robot.

        This method ensures the robot returns to its initial starting point and engages the servo to pick up the box.
        It activates the power board to secure the box and logs the action.

        Raises:
            RuntimeError: If the servo or power board fails to engage.
        """
        logger.info(f"[pick_hold_box]: Picking up box.")

        for i in range(8):
            self.stepperuino_write(data="+")
        self.sleep(3)

        self.stepperuino_write(data="d")
        self.sleep(3)

        for i in range(8):
            self.stepperuino_write(data="-")
        self.sleep(3)

        self.stepperuino_write(data="u")
        self.sleep(3)

    def drop_box(self):
        logger.info(f"[drop_box]: Dropping box.")
        for i in range(8):
            self.stepperuino_write(data="+")
        self.sleep(3)

    def hardward_initialisation(self) -> None:
        try:
            logger.info("Hardware Initialisation")
            self.stepperuino_write(data="u")
            self.sleep(3)

            for i in range(8):
                self.stepperuino_write(data="-")
            self.sleep(3)
        except:
            logger.warning("[olavesRobot - hardward_initialisation] Error during hardward_initialisation. (Except-branch of try-except statement)")

    def stepperuino_write(self, data: str) -> None:
        self.raw_serial_devices["43439353536351815121"].write(data.encode())
