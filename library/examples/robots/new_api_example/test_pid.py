import time

from libstp import PIDController

controller = PIDController(-1.0, 0.0, 0.0)
time.sleep(0.1)
print(controller.calculate(10.0))