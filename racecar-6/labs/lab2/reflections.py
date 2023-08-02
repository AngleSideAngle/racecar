class PIDController:

	def __init__(
    	self,
    	k_p: float,
    	k_i: float,
    	k_d: float,
    	period: float,
    	setpoint: float = 0,
	) -> None:
	
		self.k_p: float = k_p
		self.k_i: float = k_i
		self.k_d: float = k_d
		self.period: float = period
		self.setpoint: float = setpoint

		self.sum: float = 0
		self.prev_error: float = 0

	def calculate(self, position: float) -> float:
		error: float = self.setpoint - position

		derivative: float = (error - self.prev_error) / self.period
		self.sum += error * self.period

		self.prev_error: float = error

		return self.k_p * error + self.k_i * self.sum + self.k_d * derivative

controller: PIDController = PIDController(k_p=0.5, k_i=-0.25, k_d=0.39, period=0.5, setpoint=960)
positions: list[tuple[float, float]] = [(0.0, 250), (0.5, 600), (1.0,  780), (1.5, 912), (2.0, 1100), (2.5, 1500), (3.0, 1300), (3.5, 1102), (4.0, 924), (4.5, 882), (5.0, 956), (5.5, 1025), (6.0, 998), (6.5, 950), (7.0, 956), (7.5, 968)]

print(*[f"u({time}) = {controller.calculate(position)}" for time, position in positions], sep="\n")
print(controller.sum)