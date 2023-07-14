import time
from typing import Optional


class PIDController:
    """
    A PID implementation with variable time intervals, using system time
    """

    def __init__(self, k_p: float, k_i: float, k_d: float) -> None:
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.prev_time = time.perf_counter()
        self.prev_error = 0
        self.setpoint = 0
        self.sum = 0

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
        dt = current_time - self.prev_time
        self.prev_time = current_time

        derivative = (error - self.prev_error) / dt
        self.sum += dt * error

        self.prev_error = error

        return self.k_p * error + self.k_i + self.sum + self.k_d * derivative

    def __repr__(self) -> str:
        return f"PID: k_p: {self.k_p}, k_i: {self.k_i}, k_d: {self.k_d}, setpoint: {self.setpoint}, error: {self.prev_error}, integral: {self.sum}"