from libstp.motor import Motor
from libstp.servo import Servo
from libstp_helpers.api.steps import Step


#class MotorRotateStep(Step):
 #   def __init__(
 #           self,
 #           motor: Union[str, Motor],
 #           sensor: Union[str, Servo],
 #           *,
 #           up_velocity_pct: float,
 #           down_velocity_pct: float,
 #           hold_speed_pct: float = 0.0,
 #           shake_interval: float = 0.10,
 #           up_time: float = 0.40,
 #           sample_period: float = 0.005,
 #           max_phase_time: float = 2.5,
 #           max_retries: int = 1,
 #   ):
 #       super().__init__()
 #       self.motor_ref = motor
 #       self.sensor_ref = sensor


#    async def run_step()