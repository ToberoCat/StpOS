import asyncio
import time
from datetime import timedelta

from libstp.servo import Servo
from libstp_helpers.utility import to_task
from libstp_helpers.utility.math import ease_in_ease_out

servo = Servo(0)
servo.enable()

async def main():
    servo.set_position(2047)
    await asyncio.sleep(1)

    await to_task(servo.slowly_set_position(0, timedelta(seconds=10), ease_in_ease_out))

if __name__ == "__main__":
    asyncio.run(main())