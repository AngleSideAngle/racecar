import time


class PIDController:
    def __init__(self, p, i, d) -> None:
        self.k_p = p
        self.k_i = i
        self.k_d = d
        self.last_time=time.perf_counter()
        self.setpoint=0
        self.error=0
        self.sum=0

    def set_setpoint(self,setpoint):
        self.setpoint=setpoint
    
    def calculate(self,position):
        error=self.setpoint-position
        current_time=time.perf_counter()
        dt=current_time-self.last_time
        self.last_time=current_time

        derivative=(error-self.error)/dt
        self.sum+=dt*error

        return self.k_p*error+self.k_i+self.sum +self.k_d*derivative






        