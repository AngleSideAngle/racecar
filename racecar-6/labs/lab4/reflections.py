import numpy as np
import matplotlib.pyplot as plt
import time

N = 64 # num samples
T = 1.0 / 32.0 # sample spacing
times = np.linspace(0.0, N * T, N, endpoint=False)
noise = 1.0 * np.random.normal(0.0, 0.25, len(times))

signal = 1.0 * np.sin(1.0 * 2.0 * np.pi * times) + \
         3.0 * np.sin(3.5 * 2.0 * np.pi * times) + \
         2.5 * np.sin(22.0 * 2.0 * np.pi * times)

def integrate_simpsons(include_noise: bool = False) -> float:
    """
    Performs simpson's integration on times assuming constant interval of T
    """
    func = times + noise if include_noise else times
    return (T / 3) * (func[0] + 4 * sum(func[1:N-1:2]) + 2 * sum(func[:-2:2]) + func[-1])

def integrate_trapezoid(include_noise: bool = False) -> float:
    """
    Performs trapezoid integration on times assuming constant interval of T
    """
    func = times + noise if include_noise else times
    return T * (sum(func[1:len(func)-1]) + (func[0] + func[-1]) / 2.0)

def time_function(function):
    start_time = time.perf_counter()
    value = function()
    end_time = time.perf_counter()
    return (value, end_time - start_time)

def test_integration_rule(rule):
    answer, time = time_function(rule)
    print(f"without noise: {answer} in {time} seconds")
    answer, time = time_function(lambda: rule(True))
    print(f"with noise: {answer} in {time} seconds")

print("# simpson's rule")
test_integration_rule(integrate_simpsons)
print("# trapezoid method")
test_integration_rule(integrate_trapezoid)


plt.plot(times, signal, "*-")
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.show()