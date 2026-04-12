# olaves_robot.py
from math import pi, sin, cos, atan2, sqrt
from time import time, struct_time
from typing import List, Tuple, Optional, Union
from pid import PIDController
#from zone_coords import zone_coords

import numpy as np
import sr.robot3 as sr
import random
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


class OlavesRobot:
    def __init__(self):
        self.robot = sr.Robot(raw_ports=[("43439353536351815121", 115200)])

        # Expose commonly used robot attributes directly for convenience
        self.zone = self.robot.zone
        self.camera = self.robot.camera
        self.motor_board = self.robot.motor_board
        #self.servo_board = self.robot.servo_board
        self.raw_serial_devices = self.robot.raw_serial_devices
        self._initialise_PID_controllers()

        # Arduino access — expose whichever attribute the library uses
        if hasattr(self.robot, 'arduino'):
            self.arduino = self.robot.arduino
        if hasattr(self.robot, 'arduinos'):
            self.arduinos = self.robot.arduinos
            print(self.arduinos.keys())

        logger.info("Initialising OlavesRobot")
        self.init_time: float = time()
        self._position: np.ndarray = [0, 0, 0]  # x, y, theta
        self._last_time_updated: float = 0  # seconds
        self.ticks = [0, 0]

        self.types_of_markers = {"wall": list(i for i in range(0, 20)),
            "acidic": list(i for i in range(100, 140)) + [194],
            "basic": list(i for i in range(140, 180)),
        }

        self.blocks_found = []
        

        # Might prove helpful to distinguish between physical robot and simulated robot
        # for testing
        self.is_physical_robot = True

        # Setting servo to default position for simulated robot:
        servos_installed = False
        if not self.is_physical_robot and servos_installed:
            self.servo_board.servos[0].position = 1

        # We will load any external PID tunings here and initialise controllers for them
        # This allows for continuity between different testing runs
        self._initialise_PID_controllers()
        # self._move(0.8, mult=2.5)
        # for i in range(4):
        #     self._move(1.8, mult=2.5)
        #     self._turn(pi/2)
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
            "move_pos_dist", {"p": 0.4, "i": 0.05, "d": 0.05}
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

    def _reset_PID(self) -> None:
        self.isolated_angle_pid.reset()
        self.move_dist_pid.reset()
        self.move_angle_pid.reset()
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
        self.robot.sleep(duration)

    def _turn(self, angle: float) -> None:
        """
        Performs a dead reckoning turn.

        This method should not be used outside the OlavesRobot subclass. It calculates the duration required to turn the specified angle
        based on a predefined multiplier and adjusts motor powers accordingly.

        Parameters:
            angle (float): The angle to turn in radians. Positive values indicate a left turn, while negative values indicate a right turn.
        """
        ANGLE_TO_TIME = 1.8  # Multiplier to make the turn more accurate
        direction = "left" if angle > 0 else "right"
        logger.info(f"[_turn] | Turning {direction} by {abs(angle)} radians.")

        if angle > 0:
            # Turn left
            self.power((0.49, -0.49), angle / pi * ANGLE_TO_TIME)
        else:
            # Turn right
            self.power((-0.49, 0.49), abs(angle) / pi * ANGLE_TO_TIME)
        logger.debug(f"[_turn] | Completed turn {direction}.")

    def _move(self, distance: float, mult: float = 1) -> None:
        """
        Performs a dead reckoning move.

        This method should not be used outside the OlavesRobot subclass. It calculates the duration required to move the specified distance
        based on a predefined multiplier and adjusts motor powers accordingly.

        Parameters:
            distance (float): The distance to move in meters. Positive values move the robot forward, while negative values move it backward.
        """
        DIST_TO_TIME = 5 / mult
        direction = "forward" if distance > 0 else "backward"

        if distance > 0:
            self.power((mult*0.4, mult*0.4), distance * DIST_TO_TIME)
        else:
            self.power((mult*-0.4, mult*-0.4), abs(distance) * DIST_TO_TIME)

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
        self, marker_id: int, search_angle: float = pi * 2, speed: float = 0.2, search: bool = False
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

        no_marker_count = 0
        # Check if the marker is already in view.
        seen_markers = self.camera.see()
        markers = [m for m in seen_markers if m.id == marker_id]
        if markers:
            marker = sorted(markers, key=lambda x: x.position.distance)[0]
            logger.info(
                f"[get_marker_by_id] Marker {marker_id} found in initial check."
            )
            return marker

        if search:
            logger.info(
                f"[get_marker_by_id] Marker {marker_id} not visible initially. Initiating search..."
            )

            turned_angle = 0.0
            # Rotate in increments until we've turned the maximum allowed angle.
            while turned_angle < search_angle:
                # Rotate the robot slightly; adjust the time (0.5 seconds here) as needed.
                self._turn(pi / 6)
                self.sleep(0.3)
                turned_angle += pi / 6

                # Check the camera feed after the turn.
                seen_markers = self.camera.see()
                markers = [m for m in seen_markers if m.id == marker_id]

                if (len(seen_markers) == 0) and search:
                    no_marker_count += 1 
                    if no_marker_count >= 5:
                        self._move(-0.1)
                else:
                    no_marker_count = 0


                if markers:
                    marker = sorted(markers, key=lambda x: x.position.distance)[0]
                    logger.info(
                        f"[get_marker_by_id] Marker {marker_id} found after rotating {turned_angle}."
                    )
                    self.power((0.0, 0.0), 0.0)  # Stop the robot.
                    return marker

            logger.info(
                f"[get_marker_by_id] Could not find marker {marker_id} after rotating {search_angle:.2f} radians."
            )
        else:
            logger.info(
            f"[get_marker_by_id] Could not find marker {marker_id}."
        )
        # Stop the robot if not found
        self.power((0.0, 0.0), 0.0)
        
        return None

    def get_marker_by_idrange(
        self, marker_id_list: list, search_angle: float = pi * 2, speed: float = 0.2, is_coward: bool = True) -> Optional[sr.marker]:
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
        seen_markers = self.camera.see()
        markers = [m for m in self.camera.see() if m.id in marker_id_list]
        if (len(seen_markers) == 0) and is_coward:
            self._move(-0.1)
            self._turn(pi/6)
        if markers:
            marker = sorted(markers, key=lambda x: x.position.distance)[0]
            return marker

        turned_angle = 0.0
        # Rotate in increments until we've turned the maximum allowed angle.
        while turned_angle < search_angle:
            # Rotate the robot slightly; adjust the time (0.5 seconds here) as needed.
            self._turn(pi / 6)
            turned_angle += pi / 8

            # Check the camera feed after the turn.
            seen_markers = self.camera.see()
            markers = [m for m in self.camera.see() if m.id in marker_id_list]
            if (len(seen_markers) == 0) and is_coward:
                self._move(-0.1)
            if markers:
                marker = sorted(markers, key=lambda x: x.position.distance)[0]
                self.power((0.0, 0.0), 0.0)  # Stop the robot.
                return marker

        # If the search completes without finding the marker, stop the robot.
        self.power((0.0, 0.0), 0.0)
        return None
    
    def WallMarkerzone(self, zone: int) -> list:
        '''
        Figures out which markers are in its zone (may need to be updated every year)
        Parameters:
            zone (int): The zone the robot is in, from 0 to 3
        '''
        if zone == 0:
            return [0,19,18]
        elif zone == 1:
            return [3,4,5]
        elif zone == 2:
            return [8,9,10]
        elif zone == 3:
            return [13,14,15]
        else:
            logger.error(f"Invalid zone: {zone}. Zone must be between 0 and 3.")
            raise ValueError("Zone must be an integer between 0 and 3 inclusive.")

    def _turn_to_marker(self, marker_id: int) -> None:
        """
        Re-orients the robot to face towards a specified marker.

        Parameters:
            marker_id (int): The unique identifier of the target marker.
        """
        logger.info(f"[_turn_to_marker] | Turning to marker with id {marker_id}")

        ANGLE_TOLERANCE = 0.05
        ANGLE_TO_TIME = 0.1

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
        max_lost_frames = 4

        while abs(angle_error) > ANGLE_TOLERANCE and iterations < 100:
            angle_correction = self.isolated_angle_pid.getCorrection(angle_error)

            logger.debug(
                f"[_turn_to_marker] | Angle Correction: {angle_correction:.3f}"
            )

            left_power = -angle_correction
            right_power = +angle_correction

            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.1, center=0
            )

            logger.debug(
                f"[_turn_to_marker] | Left Power: {left_power:.3f}, Right Power: {right_power:.3f}"
            )

            time_step = min(
                ANGLE_TO_TIME,
                max((init_angle - angle_error) * ANGLE_TO_TIME, ANGLE_TO_TIME * 0.25),
            )

            self.power((left_power, right_power), time_step, off=False)

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

        self.power((0.0, 0.0), 0.0)
        logger.debug("[_turn_to_marker] | Completed or aborted turning to marker.")

    def _align_with_marker(self, marker, target_marker_pos: np.ndarray) -> None:
        """
        Aligns the robot with the specified marker.

        Parameters:
            marker (sr.Marker): The marker object to align with.
            target_marker_pos (np.ndarray): The position of the target marker.
        """
        target_position = (
            target_marker_pos[0] * cos(target_marker_pos[2]),
            target_marker_pos[1] * sin(target_marker_pos[2]),
        )

        self.move_pos(target_position=target_position)

        self.set_robot_orientation(
            target_orientation=self._restrict_angle_range(target_marker_pos[2] + pi)
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

        Parameters:
            wheels (Tuple[float, float]):
                - First element controls the left wheel's power.
                - Second element controls the right wheel's power.
            duration (float): The duration in seconds for which the motors should run.
            off (bool, optional): Whether to turn off the motors after the duration. Defaults to True.
        """
        #corrected_wheels = self._wonky_wheels_correction(wheels)
        corrected_wheels = wheels
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
        min_power: float = 0.15,
    ) -> tuple[float, float]:
        """
        Scales the two motor power values to ensure balanced and stable movement.

        Parameters:
            left (float): Desired power level for the left motor. Range: [-1.0, 1.0].
            right (float): Desired power level for the right motor. Range: [-1.0, 1.0].
            max_std (float, optional): Maximum allowed standard deviation between motor powers. Defaults to 3.0.
            center (float, optional): Target mean value for the motor powers after scaling. Defaults to 0.45.
            min_power (float, optional): Minimum non-zero power to apply to each motor. Defaults to 0.15.

        Returns:
            Tuple[float, float]: The scaled (left, right) motor power values.
        """
        logger.info(
            f"[_scale] | Scaling motor powers around {center}, with a max spread of {max_std}. Left Power: {left}, Right Power: {right}"
        )
        if min_power < 0 or min_power > 1.0:
            logger.error(
                f"Invalid min_power value: {min_power}. Must be between 0 and 1."
            )
            raise ValueError("min_power must be between 0 and 1.")

        left, right = self._apply_min_power(left, right, min_power)

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
            if 0.0 < val < min_power:
                return min_power
            elif -min_power < val < 0.0:
                return -min_power
            return val

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
            left = apply_min(left)
            right = apply_min(right)
            return (left, right)

    def _wonky_wheels_correction(
        self, wheels: Tuple[float, float]
    ) -> Tuple[float, float]:
        """
        Applies correction to the wheel power values.

        For the virtual league, no correction is applied.

        Parameters:
            wheels (Tuple[float, float]): The raw wheel power values as (left, right).

        Returns:
            Tuple[float, float]: The corrected wheel power values.
        """
        # logger.warning(
        #     "_wonky_wheels_correction is not implemented for the virtual league."
        # )
        return wheels

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

        Parameters:
            marker (Union[sr.Marker, int]): The marker object or raw marker ID integer.

        Returns:
            Optional[str]: The type of marker ('wall', 'pallet0'-'pallet3', 'highrise', or None).
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


    def move_to_marker(self, marker_id: int, random_walk: bool = False) -> bool:
        """
        Moves the robot to a specified marker using PID controllers.

        Parameters:
            marker_id (int): The ID of the target marker.
        """
        logger.info(f"[move_to_marker] | Moving to Marker: {marker_id}")

        DIST_TOLERANCE = 0.12  # meters
        ANGLE_TOLERANCE = 0.1  # radians
        DIST_TO_TIME = 0.07  # Base duration for movement

        marker = self.get_marker_by_id(marker_id=marker_id, search=True)
        if not marker:
            logger.error(
                f"[move_to_marker] Marker {marker_id} not found. Aborting turn."
            )
            return False

        init_dist = marker.position.distance / 1000.0  # Convert mm to meters

        dist_error = init_dist
        angle_error = marker.position.horizontal_angle

        prev_dist = None

        logger.debug(
            f"[move_to_marker] | Initial Distance Error: {dist_error:.3f} m, Initial Angle Error: {angle_error:.3f} rad"
        )

        iterations = 0
        consecutive_lost_frames = 0
        max_lost_frames = 5
        stationary_consecutive = 0

        self._reset_PID()

        while (
            (dist_error > DIST_TOLERANCE or abs(angle_error) > ANGLE_TOLERANCE)
            and iterations < 50
            and consecutive_lost_frames < max_lost_frames
        ):  
            prev_dist = dist_error
            dist_correction = -self.move_dist_pid.getCorrection(dist_error)
            angle_correction = self.move_angle_pid.getCorrection(angle_error)

            logger.debug(
                f"[move_to_marker] | Distance Correction: {dist_correction:.3f}, Angle Correction: {angle_correction:.3f}"
            )

            left_power = dist_correction - angle_correction
            right_power = dist_correction + angle_correction

            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.15
            )

            logger.debug(
                f"[move_to_marker] | Left Power: {left_power:.3f}, Right Power: {right_power:.3f}"
            )

            time_step = min(
                DIST_TO_TIME,
                max((init_dist - dist_error) * DIST_TO_TIME, DIST_TO_TIME * 0.1),
            )

            self.power((left_power, right_power), time_step, off=False)

            marker = self.get_marker_by_id(marker_id)
            if not marker:
                
                if consecutive_lost_frames >= max_lost_frames:    
                    logger.error(
                        "[move_to_marker] | Marker lost too many times, aborting."
                    )
                   
                    return False
                if dist_error <= 3 * DIST_TOLERANCE:
                    logger.error(
                        f"[move_to_marker] | Lost Marker at Distance {dist_error}, moving forward this amount."
                    )
                    if marker_id not in self.types_of_markers["wall"]:
                        self._move(0.3)
                        return self._move_to_marker_finalisation()
                    self.power((0.0, 0.0), 0.0)
                    
                    return True
                logger.warning(
                    f"[move_to_marker] | Lost marker {marker_id}. Conducting search."
                )
                consecutive_lost_frames += 1
                continue
            else:
                prev_dist = dist_error
                angle_error = marker.position.horizontal_angle
                dist_error = marker.position.distance / 1000.0
                logger.debug(
                    f"[move_to_marker] Updated Distance Error: {dist_error:.3f} m, Updated Angle Error: {angle_error} rads."
                )
                if abs(dist_error - prev_dist) <= 1e-2:
                    stationary_consecutive += 1
                    if stationary_consecutive >= 5:
                        self._move(-0.5)
                        self._turn(-pi/5)
                else:
                    stationary_consecutive = 0
                
            iterations += 1
        
        self.power((0.0, 0.0), 0.0)
        return False

    def _move_to_marker_finalisation(self) -> bool:
        """
        Finalises the motion for move_to_marker using the ultrasound sensor.
        """
        logger.info(
            f"[_move_to_marker_finalisation] | Minimising distance between robot and marker in the absence of camera feedback"
        )
        DIST_TOLERANCE = 0.03  # meters

        init_dist = (
            self.arduino.ultrasound_measure(7, 6) / 1000.0
        )  # Convert mm to meters

        dist_error = init_dist

        logger.debug(
            f"[_move_to_marker_finalisation] | Initial Distance Error: {dist_error:.3f} m"
        )

        iterations = 0
        while (dist_error > DIST_TOLERANCE) and (dist_error < 0.3) and iterations != 30:
            

            self._move(dist_error * 0.7)

            dist_error = self.arduino.ultrasound_measure(7, 6) / 1000.0
            iterations += 1
            logger.debug(
                f"[_move_to_marker_finalisation] | Moving forward {dist_error}m"
            )

        if dist_error >= 0.30:
            self._move(-0.5)
            return False

        if iterations >= 30:
            self.power((0.0, 0.0), 0.0)
            return False

        logger.debug(
            "[_move_to_marker_finalisation] | Marker in position. Exiting Movement"
        )

        return True

    def move_pos(self, target_position: Tuple[float, float]) -> None:
        """
        Moves the robot to a specific (x, y) position using PID controllers.

        Parameters:
            target_position (Tuple[float, float]): The (x, y) coordinates to move the robot to.
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

        init_dist = distance
        dist_error = init_dist

        angle_error = angle - phi
        angle_error = (angle_error + pi) % (2 * pi) - pi

        logger.debug(
            f"[move_pos] | Initial Distance Error: {dist_error:.3f} m, Initial Angle Error: {angle_error:.3f} rad"
        )

        iterations = 0

        while (
            dist_error > DIST_TOLERANCE or abs(angle_error) > ANGLE_TOLERANCE
        ) and iterations != 1000:
            dist_correction = -self.move_dist_pid.getCorrection(dist_error)
            angle_correction = self.move_angle_pid.getCorrection(angle_error)

            logger.debug(
                f"[move_pos] | Distance Correction: {dist_correction:.3f}, Angle Correction: {angle_correction:.3f}"
            )

            left_power = dist_correction - angle_correction
            right_power = dist_correction + angle_correction

            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.15
            )

            logger.debug(
                f"[move_pos] | Left Power: {left_power:.3f}, Right Power: {right_power:.3f}"
            )

            time_step = min(
                max((dist_error / init_dist) * DIST_TO_TIME, 0.5 * DIST_TO_TIME),
                DIST_TO_TIME,
            )

            self.power((left_power, right_power), time_step, off=False)

            current_position = self.get_pos()

            current_coords, phi = current_position[:2], current_position[2]

            destination_vector = np.array(target_position) - current_coords
            dist_error = np.linalg.norm(destination_vector)
            desired_angle = atan2(destination_vector[1], destination_vector[0])
            angle_error = desired_angle - phi
            angle_error = (angle_error + pi) % (2 * pi) - pi

            logger.debug(
                f"[move_pos] | Updated Distance Error: {dist_error:.3f} m, Updated Angle Error: {angle_error:.3f} rad"
            )

            iterations += 1

        self.power((0.0, 0.0), 0.0)
        logger.info("Position Reached")

    def camera_pos(self) -> Optional[Tuple[float, float]]:
        """
        Estimate position from camera alone using known field markers.

        DOES NOT WORK YET, DON'T USE
        """
        logger.info(f"[camera_pos] | Using camera to find location")
        markers = self.camera.see()
        zone_coordinates = self.zone_coords

        visible_known_markers = [m for m in markers if m.id in range(27)]
        if not visible_known_markers:
            return self._encoder_pos()
        if len(visible_known_markers) < 2:
            return self._encoder_pos()

        marker = sorted(visible_known_markers, key=lambda x: x.position.distance)[0]
        a, b, c, d = marker[0][1], marker[0][2], marker[1][1], marker[1][2]
        theta, alpha = marker.orientation.yaw(marker[0][0]), marker.orientation.yaw(
            marker[1][0]
        )

        import math
        x = ((math.tan(theta) * a) + b - a + (math.tan(alpha) * c)) / (
            (math.tan(theta)) + (math.tan(alpha))
        )
        y = math.tan(theta) * x - math.tan(theta) * a + b
        return (x, y)

    def pick_hold_box(self):
        """
        Picks up and holds the box.

        Assumes that the box is directly ahead of the robot.
        """

        self._move(-0.08)
        logger.info(f"[pick_hold_box]: Picking up box.")


        for i in range(8):
            self.stepperuino_write(data="+")
        self.sleep(3)
        logger.info(f"[pick_hold_box]: Opening Up")

        for i in range(15):
            self.stepperuino_write(data="@")
        
        logger.info(f"[pick_hold_box]: Going Down")
        self.sleep(3)

        for i in range(8):
            self.stepperuino_write(data="-")
        logger.info(f"[pick_hold_box]: Going In")
        self.sleep(3)

        self.stepperuino_write(data="u")
        logger.info(f"[pick_hold_box]: Going Up")
        self.sleep(3)

    def drop_box(self):
        logger.info(f"[drop_box]: Dropping box.")
        for i in range(8):
            self.stepperuino_write(data="+")
        self.sleep(3)

        for i in range(8):
            self.stepperuino_write(data="-")
        logger.info(f"[drop_box]: Reseting Arms")
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
