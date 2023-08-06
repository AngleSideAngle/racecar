"""
Group 6's library of control code includes utilities we use in controlling our robot
"""

import time
from typing import Optional, NamedTuple

class PIDConstants(NamedTuple):
    """
    PID Constants
    """

    k_p: float
    k_i: float
    k_d: float

    def __repr__(self) -> str:
        return f"PIDConstants(k_d={self.k_d}, k_i={self.k_i}, k_d={self.k_d})"

    def __mul__(self, __value):
        return PIDConstants(__value * self.k_p, __value * self.k_i, __value * self.k_d) # type: ignore

    def __rmul__(self, __value):
        return self.__mul__(__value)

class PIDController:
    """
    A PID implementation with variable time intervals, using system time
    """

    def __init__(
        self,
        constants: PIDConstants,
        setpoint: float = 0,
        min_output: Optional[float] = None,
        max_output: Optional[float] = None
    ) -> None:
        self.constants = constants

        self.prev_time = time.perf_counter()
        self.prev_error = 0
        self.setpoint = setpoint
        self.sum = 0

        self.min_output = min_output
        self.max_output = max_output

    def calculate(
        self,
        position: float,
        setpoint: Optional[float] = None
    ) -> float:
        """
        Calculates control output to minimize error based on the PID algorithm
        """

        if setpoint:
            self.setpoint = setpoint

        error = self.setpoint - position

        current_time = time.perf_counter()
        delta_time = current_time - self.prev_time
        self.prev_time = current_time

        derivative = (error - self.prev_error) / delta_time
        self.sum += delta_time * error

        self.prev_error = error

        control_effort = self.constants.k_p * error + self.constants.k_i * self.sum + self.constants.k_d * derivative

        if self.min_output:
            control_effort = max(control_effort, self.min_output)

        if self.max_output:
            control_effort = min(control_effort, self.max_output)

        return control_effort

    def __repr__(self) -> str:
        return f"PID: constants: {self.constants}, setpoint: {self.setpoint}, error: {self.prev_error}, integral: {self.sum}"


class RateLimiter:
    """
    A rate limited value that accounts for time
    """

    value: float
    rate: float
    prev_time: float

    def __init__(self, rate: float, value: float = 0) -> None:
        """
        value: initial float
        rate: maximum Δ value / Δ time
        """

        self.rate = rate
        self.value = value
        self.prev_time = time.perf_counter()

    def update(self, new_value: float) -> float:
        """
        Constrains the input within the allowed rate
        """

        current_time = time.perf_counter()
        delta_time = current_time - self.prev_time
        self.value += clamp(new_value - self.value, -self.rate * delta_time, self.rate * delta_time)
        self.prev_time = current_time
        return self.value

    def reset(self, starting_value: float = 0) -> None:
        """
        Resets the limiter to the provided starting value
        """

        self.value = starting_value
        self.prev_time = time.perf_counter()


class Debouncer:
    """
    Debounces falling signal
    """

    def __init__(self, baseline: bool, debounce_time: float, starting_value: bool = False) -> None:
        self.baseline = baseline
        self.debounce_time = debounce_time
        self.start_time = time.perf_counter()

    def update(self, value: bool) -> bool:
        """
        Updates debouncer
        """

        if value == self.baseline:
            self.start_time = time.perf_counter()

        if time.perf_counter() - self.start_time >= self.debounce_time:
            return value

        return self.baseline



def clamp(value: float, lower_bound: float, upper_bound: float) -> float:
    """
    Clamps inputted value between upper and lower bounds, inclusively
    """

    return min(max(value, lower_bound), upper_bound)
