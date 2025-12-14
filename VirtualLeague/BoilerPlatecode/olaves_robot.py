# olaves_robot.py

import math
from math import pi, sin, cos, atan2, sqrt, tan, floor, degrees, atan
import logging
from typing import List, Tuple, Optional, Union, Callable
#Sympy here if needed, but preferable we do not use it for comp doesn't have it preinstalled - will delete if doesn't end up being used
#from sympy import Matrix,linsolve,symbols

import numpy as np
import sr.robot3 as sr
from pid import PIDController

logger = logging.getLogger(__name__)


class OlavesRobot(sr.Robot):
    def __init__(self):
        super().__init__()
        logger.info("Initialising OlavesRobot")
        self.init_time: float = self.time()
        self._position: np.ndarray = [0, 0, 0]  # x, y, theta
        self._last_time_updated: float = 0  # seconds
        self.ticks = [0, 0]
        arduino = self.arduinos["Arduino1"]
        my_servo_board = self.servo_board
        my_power_board = self.power_board

    def get_comp_time(self) -> float:
        """
        Returns the time elapsed since the robot was initialised.

        Computes the difference between the current time and the initialisation time.

        Returns:
            float: Elapsed time in seconds.
        """
        elapsed_time = self.time() - self.init_time
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

    def _update_time(self):
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

    def encoder_pos(
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

        # MATHS FROM: https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf

        logger.info("Calculating position from encoder ticks.")

        TICKS_PER_ROT = 2880
        WHEEL_RADIUS = 0.072
        ROBOT_WIDTH = 0.42
        HALF_ROBOT_WIDTH = ROBOT_WIDTH / 2

        # Get the wheel ticks since last check
        ticks = self.get_motor_ticks()
        logger.debug(f"Encoder ticks received: {ticks}.")

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

        logger.info(
            f"Encoder position updated to [x={pos[0]:.3f}, y={pos[1]:.3f}, \n"
            f"theta={pos[2]:.3f}] after {delta_t:.3f}s."
        )

        return pos

    def sleep(self, duration: float) -> None:
        """
        Pauses execution for a specified duration.

        Acts as time.sleep, but tailored for the simulation environment where robot.sleep() may not function as expected.

        Parameters:
            duration (float): The amount of time to sleep in seconds.
        """
        logger.debug(f"Sleeping for {duration} seconds.")
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
        logger.info(f"Turning {direction} by {abs(angle)} radians.")

        if angle > 0:
            # Turn left
            self.power((0.4, -0.4), angle / pi * ANGLE_TO_TIME)
        else:
            # Turn right
            self.power((-0.4, 0.4), abs(angle) / pi * ANGLE_TO_TIME)
        logger.info(f"Completed turn {direction}.")

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
        logger.info(f"Moving {direction} by {abs(distance)} meters.")

        if distance > 0:
            self.power((0.4, 0.4), distance * DIST_TO_TIME)
        else:
            self.power((-0.4, -0.4), abs(distance) * DIST_TO_TIME)
        logger.info(f"Completed move {direction}.")

    def get_marker_orientation(self, marker_id: int) -> float:
        """
        Returns the absolute orientation of a specified marker.

        This method is not needed for the virtual league and is intentionally left unimplemented.

        Assumptions:
            - To be based on new positioning system proposed by Shehryer and Francis

        Parameters:
            marker_id (int): The ID of the marker.

        Raises:
            NotImplementedError: Always raised as this method is not required for the virtual league.
        """
        logger.warning(
            "get_marker_orientation is not implemented for the virtual league."
        )
        raise NotImplementedError(
            "get_marker_orientation is not needed for the virtual league."
        )

    def _get_marker_by_id(self, marker_id: int) -> sr.marker:
        """
        Retrieves a marker by its ID.

        Searches the robot's camera feed for a marker with the specified ID.

        Parameters:
            marker_id (int): The ID of the marker to retrieve.

        Returns:
            sr.Marker: The marker object corresponding to the given ID.

        Raises:
            ValueError: If no marker with the specified ID is found.
        """
        marker = next((m for m in self.camera.see() if m.id == marker_id), None)
        if marker is None:
            logger.error(f"Marker with ID {marker_id} not found.")
            raise ValueError(f"Marker with ID {marker_id} not found.")
        logger.debug(f"Marker found: {marker}")
        return marker

    def _turn_to_marker(self, target_marker_id: int) -> None:
        """
        Re-orients the robot to face towards a specified marker.

        **Note:** This method is not required for the virtual league and remains unimplemented.

        Assumes that the robot is positioned directly in front of the marker.

        Parameters:
            target_marker_id (int): The unique identifier of the target marker.

        Raises:
            NotImplementedError: This method is intentionally unimplemented for the virtual league.
        """
        logger.warning("_turn_to_marker is not implemented for the virtual league.")
        raise NotImplementedError(
            "_turn_to_marker is not needed for the virtual league."
        )

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
        logger.warning("_align_with_marker is not implemented for the virtual league.")
        raise NotImplementedError(
            "_align_with_marker is not needed for the virtual league."
        )

    def get_marker_pos(self, marker_id: int) -> Tuple[float, float]:
        # NEEDS TO BE TESTED - IMPLEMENTED BY AARIT
        """
        Calculates the absolute position of a marker based on its relative position to the robot.

        Parameters:
            marker_id (int): The ID of the marker.

        Returns:
            Tuple[float, float]: A tuple containing the (x, y) coordinates of the marker.

        Raises:
            ValueError: If the marker with the specified ID is not found.
        """
        logger.debug(f"Calculating position for marker ID {marker_id}.")
        marker = self._get_marker_by_id(marker_id)
        # TODO: Implement position calculation logic based on marker's relative position
        # Example:
        # x = ...  # Calculate x-coordinate
        # y = ...  # Calculate y-coordinate
        # return (x, y)
        markers = self.camera.see()
        for marker in markers:
            if marker_id == marker.id:
                markerOrientation = marker.orientation.yaw
                MarkerDistance = marker.position.distance / 1000

        #To check if the marker is actually visible or not
        if markerOrientation == None or MarkerDistance == None:
            raise ValueError(f"Marker with ID {marker_id} not found.")


        #Retrieves the robot current position to estimate where the marker is relative to it
        Robot_current_pos = (
            self.get_pos()
        )  # Here I am assuming that the angle is a bearing in radians
        robot_x, robot_y, robot_theta = (
            Robot_current_pos[0],
            Robot_current_pos[1],
            Robot_current_pos[2],
        )
        # Idea here is to utilise the cosine rule by making an isosceles triangle with the 'A' angle being marker orientation
        x_vector = (
            math.sqrt(
                2 * MarkerDistance**2
                - (MarkerDistance**2) * math.cos(markerOrientation)
            )
            / 2
        )
        y_vector = math.sqrt((MarkerDistance**2) - (x_vector**2))
        # Checks whether the actual marker is behind the robot or not, by looking at the orientation of the robot and check what the orientation of the marker is to add either negative x vector or positive x vector
        if (robot_theta > (1.5)*pi and robot_theta < 2*pi) or (
            robot_theta > 0 and robot_theta < 0.5*pi
        ):
            if marker.orientation.yaw > 0:
                x_coord = robot_x - x_vector
                y_coord = y_vector + robot_y
            else:
                x_coord = x_vector + robot_x
                y_coord = y_vector + robot_y
        else:
            if marker.orientation.yaw > 0:
                x_coord = robot_x + x_vector
                y_coord = y_vector - robot_y
            else:
                x_coord = x_vector - robot_x
                y_coord = y_vector - robot_y
        return (x_coord, y_coord)

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
        logger.debug("Estimating robot's position.")
        # TODO: Implement position estimation logic here
        raise NotImplementedError("get_pos method needs to be implemented.")

    def set_robot_orientation(self, theta: float, phi: float) -> None:
        """
        Sets the robot's orientation to a specified angle using PID control.

        The method adjusts the robot's orientation from the current angle `phi` to the target angle `theta`, minimising the error through PID control.

        Parameters:
            theta (float): The target orientation angle in radians.
            phi (float): The current orientation angle in radians.

        Raises:
            NotImplementedError: Indicates that the method's implementation is pending.
        """
        logger.info(f"Setting robot orientation from {phi} to {theta} radians.")
        # TODO: Implement orientation setting logic using PID controller - need to check if necessary for virtual league
        raise NotImplementedError(
            "set_robot_orientation method needs to be implemented."
        )

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
        logger.info(
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
        logger.debug(f"Clamping value {value} to {clamped_value}.")
        return clamped_value

    def _scale(
        self,
        left: float,
        right: float,
        max_std: float = 3,
        center: float = 0.45,
        min_power: float = 0.15,
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
        if min_power < 0 or min_power > 1.0:
            logger.error(
                f"Invalid min_power value: {min_power}. Must be between 0 and 1."
            )
            raise ValueError("min_power must be between 0 and 1.")

        # Compute mean
        mu = (left + right) / 2.0

        # Compute population standard deviation for two points
        sd = sqrt(((left - mu) ** 2 + (right - mu) ** 2) / 2.0)

        # Scale down if std exceeds max_std
        if sd > max_std and sd > 1e-9:
            scale_factor = max_std / sd
            left = mu + scale_factor * (left - mu)
            right = mu + scale_factor * (right - mu)

        # Shift mean to center
        new_mu = (left + right) / 2.0
        shift = center - new_mu
        left += shift
        right += shift

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

        # TODO: fix to allow turning
        # Right now it sets the max power to 0.15, which means negative powers are disallowed, preventing turning ability

        def apply_min(val):
            if 0.0 < val < min_power:
                return min_power
            elif -min_power < val < 0.0:
                return -min_power
            return val

        return (apply_min(left), apply_min(right))

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

    def get_marker_type(self, marker: Union[sr.marker, int]) -> Optional[str]:
        """
        Determines the type of a marker based on its ID.

        Categorises markers into predefined types such as walls, acids, bases markers based on their ID ranges.

        Parameters:
            marker (Union[sr.Marker, int]): The marker object or raw marker ID integer.

        Returns:
            

        Raises:
            ValueError: If the provided marker is neither an sr.Marker instance nor an integer.
        """
        if isinstance(marker, sr.Marker):
            marker_id = marker.id
        elif isinstance(marker, int):
            marker_id = marker
        else:
            logger.error(f"Invalid marker type: {type(marker)}")
            raise ValueError(f"Invalid marker type: {type(marker)}")

        logger.debug(f"Determining type for marker ID: {marker_id}")

        #TODO: Update these ranges based on actual marker ID assignments according to comp rules

    

    def pick_hold_box(self):
        """
        Picks up and holds the box.

        This method ensures the robot returns to its initial starting point and engages the servo to pick up the box.
        It activates the power board to secure the box and logs the action.

        Raises:
            RuntimeError: If the servo or power board fails to engage.
        """
        #TODO: Check if this works
        try:
            # Ensure the robot is at the initial starting point
            self.servo_board.servos["SERVO"].position = -1
            logger.info("Servo positioned to pick up the box.")

            markers = self.camera.see()
            target_id = 123  # Example marker ID to look for
            for marker in markers:
                if marker.id == target_id:
                    marker_orientation = marker.orientation.yaw
                    logger.debug(f"Marker orientation yaw: {marker_orientation}")

            distance_m = self.arduino.ultrasound_measure(2, 3) * 1e-3
            logger.debug(f"Measured distance: {distance_m} meters.")

            if distance_m == 0:
                self.power_board.outputs["OUT_H0"].power_on()
                self.sleep(3)
                logger.info("Box secured by activating power board.")
        except Exception as e:
            logger.error(f"Failed to pick up and hold the box: {e}")
            raise RuntimeError("Box pickup failed.") from e

    def move_to_marker(self, marker_id: int) -> None:
        """
        Moves the robot to a specified marker using PID controllers.

        Parameters:
            marker_id (int): The ID of the target marker.
        """
        logger.info(f"Moving to Marker: {marker_id}")

        DIST_TOLERANCE = 0.2  # meters
        ANGLE_TOLERANCE = 0.1  # radians
        DIST_TO_TIME = 0.5  # Base duration for movement

        markers = [m for m in self.camera.see() if m.id == marker_id]
        if not markers:
            logger.error(f"Marker {marker_id} not seen initially.")
            # TODO: Add some handling logic for when a marker can't be seen
            return

        marker = markers[0]
        init_dist = marker.position.distance / 1000.0  # Convert mm to meters

        dist_error = init_dist
        angle_error = marker.position.horizontal_angle

        logger.debug(
            f"Initial Distance Error: {dist_error:.3f} m, Initial Angle Error: {angle_error:.3f} rad"
        )

        # Initialise PID controllers
        # Distance PID: Only P and I to prevent aggressive changes
        dist_pid = PIDController(
            sp=0.0, p=0.05, i=0.01, d=0.0, integrator_min=-5.0, integrator_max=5.0
        )

        # Angle PID: P and D for responsive turning with damping
        angle_pid = PIDController(
            sp=0.0, p=0.15, i=0.01, d=0.05, integrator_min=-5.0, integrator_max=5.0
        )

        while dist_error > DIST_TOLERANCE or abs(angle_error) > ANGLE_TOLERANCE:
            # Calculate PID corrections
            dist_correction = -dist_pid.getCorrection(dist_error)
            angle_correction = angle_pid.getCorrection(angle_error)

            logger.debug(
                f"Distance Correction: {dist_correction:.3f}, Angle Correction: {angle_correction:.3f}"
            )

            # Convert corrections into left/right motor power
            left_power = dist_correction - angle_correction
            right_power = dist_correction + angle_correction

            # Apply scaling with minimum power
            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.15
            )

            logger.debug(
                f"Left Power: {left_power:.3f}, Right Power: {right_power:.3f}"
            )

            # Determine dynamic time_step based on distance error
            # This can help in smoother control as the robot approaches the target
            time_step = min(
                max((dist_error / init_dist) * DIST_TO_TIME, 0.5 * DIST_TO_TIME),
                DIST_TO_TIME,
            )

            # Apply motor power
            self.power((left_power, right_power), time_step, off=False)

            # Update marker information
            markers = [m for m in self.camera.see() if m.id == marker_id]
            if markers:
                marker = markers[0]
            else:
                logger.error(f"Marker {marker_id} lost. Exiting movement.")
                self.power((0.0, 0.0), 0.0)
                return

            # Update errors
            dist_error = marker.position.distance / 1000.0
            angle_error = marker.position.horizontal_angle

            logger.debug(
                f"Updated Distance Error: {dist_error:.3f} m, Updated Angle Error: {angle_error:.3f} rad"
            )

        # Stop the robot once target is reached
        self.power((0.0, 0.0), 0.0)
        logger.info("Marker Reached")

    def move_pos(self, target_position: Tuple[float, float]) -> None:
        """
        Moves the robot to a specific (x, y) position using PID controllers for angle and distance.

        The method calculates the required adjustments in motor power to navigate the robot to the desired position accurately.

        Parameters:
            target_position (Tuple[float, float]): The (x, y) coordinates to move the robot to.

        Raises:
            NotImplementedError: Indicates that the method's implementation is pending.
        """
        logger.info(f"Moving to position {target_position}.")

        DIST_TOLERANCE = 0.2  # meters
        ANGLE_TOLERANCE = 0.1  # radians
        DIST_TO_TIME = 0.5  # Base duration for movement

        current_position = self.get_pos()

        current_coords, phi = current_position[:2], current_position[2]

        destination_vector = target_position - current_coords
        angle = atan2(destination_vector[1], destination_vector[0])
        distance = (destination_vector[0] ** 2 + destination_vector[1] ** 2) ** 0.5

        init_dist = distance  # Convert mm to meters

        dist_error = init_dist
        angle_error = angle

        logger.debug(
            f"Initial Distance Error: {dist_error:.3f} m, Initial Angle Error: {angle_error:.3f} rad"
        )

        # Initialise PID controllers
        # Distance PID: Only P and I to prevent aggressive changes
        dist_pid = PIDController(
            sp=0.0, p=0.05, i=0.01, d=0.0, integrator_min=-5.0, integrator_max=5.0
        )

        # Angle PID: P and D for responsive turning with damping
        angle_pid = PIDController(
            sp=0.0, p=0.15, i=0.01, d=0.05, integrator_min=-5.0, integrator_max=5.0
        )

        while dist_error > DIST_TOLERANCE or abs(angle_error) > ANGLE_TOLERANCE:
            # Calculate PID corrections
            dist_correction = -dist_pid.getCorrection(dist_error)
            angle_correction = angle_pid.getCorrection(angle_error)

            logger.debug(
                f"Distance Correction: {dist_correction:.3f}, Angle Correction: {angle_correction:.3f}"
            )

            # Convert corrections into left/right motor power
            left_power = dist_correction - angle_correction
            right_power = dist_correction + angle_correction

            # Apply scaling with minimum power
            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.15
            )

            logger.debug(
                f"Left Power: {left_power:.3f}, Right Power: {right_power:.3f}"
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

            destination_vector = target_position - current_coords
            angle_error = atan2(destination_vector[1], destination_vector[0])
            dist_error = (
                destination_vector[0] ** 2 + destination_vector[1] ** 2
            ) ** 0.5

            logger.debug(
                f"Updated Distance Error: {dist_error:.3f} m, Updated Angle Error: {angle_error:.3f} rad"
            )

        # Stop the robot once target is reached
        self.power((0.0, 0.0), 0.0)
        logger.info("Position Reached")

    #def newPositioning(self):
    #TODO: Uses sympy currently, need a way to use it without sympy or just ignore completely
    '''
    Inputs: None
    Outputs: [x coordinate, y coordinate, angle] relative to its starting position
    
    '''
    #    x, y = symbols("x, y")
    #    markers = self.camera.see()

    #    equation_list = []

    #    def equationmaker(distance, xcoord , ycoord, angle):
    #        m = distance*tan(degrees(angle))
    #        c = ycoord-(m*xcoord)
    #        Eqn = [-1*m*x + y - c]
    #        equation_list.append(Eqn)

    #    def equationsolver(eqa, eqb): # fu+nction to find the common coordinate of 2 lines
    #        x = int(len(equation_list))
    #        for i in range(floor(x/2)):
    #            linsolve((eqa, eqb), x, y)    

    #    def location_finder(markers):
    #        marker_info = []

    #        for marker in markers:
    #            if int(marker.id) >= 0 and int(marker.id) <= 27:
    #                marker_info.extend([marker.position.distance, marker.position.horizontal_angle, marker.id,Marker_Dictionary[str(marker.id)][0] , Marker_Dictionary[str(marker.id)][1]])

            # for each marker find the equation of the lines from the marker to the robot
    #        xcoord_new = 0
    #        ycoord_new = 0
    #        for i in marker_info:
    #            equationmaker(i[0],i[3],i[4],i[1])
    #        for i in range(floor(len(equation_list)/2)):
    #            eq_output=equationsolver(equation_list[i], equation_list[random.choice(equation_list)])
    #            xcoord_new += eq_output[0]
    #            ycoord_new += eq_output[1]
    #        xcoord_new=xcoord_new/(floor(len(equation_list)/2))
    #        ycoord_new=ycoord_new/(floor(len(equation_list)/2))
    #        return (xcoord_new, ycoord_new )
    #        
    #    def angle_finder(xcoord, ycoord):
    #        angle = atan(ycoord/xcoord)
    #        if angle < 0:
    #            if xcoord < 0:
    #                angle = abs(angle) + 270
    #            elif xcoord >= 0:
     #               angle = abs(angle) + 90
     #       elif angle >= 0:
    #            if xcoord < 0:
    #                angle = abs(angle) + 180
    #           else: 
    #                angle = 90-abs(angle)
    #        
    #        return angle

    #    Marker_Dictionary = {
    #        "0": (-2155.5, 2875),
    #        "1": (-1437.5, 2875),
    #        "2": (-719.5, 2875),
     #       "3": (0, 2875),
     #       "4": (719.5, 2875),
     #       "5": (1437.5, 2875),
     #       "6": (2155.5, 2875),
     #       "7": (2875, 2155.5),
     #       "8": (2875, 1437.5),
     #       "9": (2875, 719.5),
     #       "10": (2875, 0),
     #       "11": (2875, -719.5),
     #       "12": (2875, -1437.5),
    #        "13": (2875, -2155.5),
    #        "14": (2155.5, -2875),
    #        "15": (1437.5, -2875),
    #        "16": (719.5, -2875),
    #        "17": (0, -2875),
    #        "18": (-719.5, -2875),
    #        "19": (-1437.5, -2875),
    #        "20": (-2155.5, -2875),
    #        "21": (-2875, -2155.5),
    #        "22": (-2875, -1437.5),
    #        "23": (-2875, -719.5),
    #        "24": (-2875, 0),
    #        "25": (-2875, 719.5),
    #        "26": (-2875, 1437.5),
    #        "27": (-2875, 2155.5),
    #    }

    #    xcoord, ycoord = location_finder(markers)
    #    angle = angle_finder(xcoord, ycoord)
    #    return [xcoord, ycoord, angle]