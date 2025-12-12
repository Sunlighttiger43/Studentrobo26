
# pid.py

from time import time


class PIDController:
    """
    A PID (Proportional, Integral, Derivative) controller with anti-windup
    and derivative filtering.
    """

    def __init__(
        self,
        sp: float,
        p: float = 0.0,
        i: float = 0.0,
        d: float = 0.0,
        integrator_min: float = -10.0,
        integrator_max: float = 10.0,
        derivative_filter_coeff: float = 0.8,
    ):
        """
        Initialises the PIDController with given gain values and parameters.

        Parameters:
            sp (float): The setpoint (target value).
            p (float, optional): Proportional gain. Defaults to 0.0.
            i (float, optional): Integral gain. Defaults to 0.0.
            d (float, optional): Derivative gain. Defaults to 0.0.
            integrator_min (float, optional): Minimum integral term. Defaults to -10.0.
            integrator_max (float, optional): Maximum integral term. Defaults to 10.0.
            derivative_filter_coeff (float, optional): Coefficient for derivative filtering (0 < coeff < 1). Defaults to 0.8.
        """
        self.sp = sp
        self.p = p
        self.i = i
        self.d = d

        # Anti-windup limits
        self.integrator_min = integrator_min
        self.integrator_max = integrator_max

        # Derivative filter coefficient
        self.derivative_filter_coeff = derivative_filter_coeff

        # Internal state
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        self.last_time = None

    def getCorrection(self, pv: float) -> float:
        """
        Computes the control signal (correction) based on the current process variable.

        Parameters:
            pv (float): The current process variable.

        Returns:
            float: The calculated correction value.
        """
        current_time = time()

        # Calculate time difference
        if self.last_time is None:
            dt = 0.0
        else:
            dt = current_time - self.last_time

        self.last_time = current_time

        # Calculate error
        error = self.sp - pv

        # Proportional term
        p_term = self.p * error

        # Integral term with anti-windup
        if dt > 0.0:
            self.integral += error * dt
            # Clamp integral to prevent windup
            self.integral = max(
                self.integrator_min, min(self.integrator_max, self.integral)
            )
        i_term = self.i * self.integral

        # Derivative term with filtering
        if dt > 0.0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        # Apply low-pass filter to derivative
        derivative_filtered = (
            self.derivative_filter_coeff * self.prev_derivative
            + (1 - self.derivative_filter_coeff) * derivative
        )
        d_term = self.d * derivative_filtered

        self.prev_derivative = derivative_filtered
        self.prev_error = error

        # PID output
        correction = p_term + i_term + d_term

        return correction

    def reset(
        self,
        sp: float | None = None,
        p: float | None = None,
        i: float | None = None,
        d: float | None = None,
    ):
        """
        Resets the PID controller state and optionally updates gains.

        Parameters:
            sp (float, optional): The new setpoint. If None, it remains unchanged.
            p (float, optional): The new proportional gain. If None, it remains unchanged.
            i (float, optional): The new integral gain. If None, it remains unchanged.
            d (float, optional): The new derivative gain. If None, it remains unchanged.
        """
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        self.last_time = None

        if sp is not None:
            self.sp = sp
        if p is not None:
            self.p = p
        if i is not None:
            self.i = i
        if d is not None:
            self.d = d
