import time

from libstp.logging import info
from libstp.sensor import MagnetoXSensor, MagnetoYSensor, MagnetoZSensor, wait_for_button_click, is_button_clicked

data_file = "magnetometer_data.csv"
frequency = 100


mag_x = MagnetoXSensor()
mag_y = MagnetoYSensor()
mag_z = MagnetoZSensor()


info("Click the button to start sampling the magnetometer")
info("Once the button is clicked, the sampling will start in 3 seconds")
info("To stop the sampling, click the button again")
info("When sampling, rotate the robot around all axes")
wait_for_button_click()

timeout = 3
for i in range(timeout):
    info(f"Calibration starting in {timeout - i} seconds")
    time.sleep(1)


info("=====================================")
info("Sampling magnetometer...")
info("=====================================")

dataset = []
start_time = time.time()
def add_data_point(x, y, z):
    dataset.append((x, y, z, time.time() - start_time))

while not is_button_clicked():
    x = mag_x.get_value()
    y = mag_y.get_value()
    z = mag_z.get_value()
    add_data_point(x, y, z)

    if len(dataset) % 100 == 0:
        info(f"Collected {len(dataset)} data points")

    time.sleep(1 / frequency)

info(f"Sampling done - Storing {len(dataset)} data points to {data_file}")
with open(data_file, "w") as f:
    f.write("x,y,z,time\n")
    for x, y, z, t in dataset:
        f.write(f"{x},{y},{z},{t}\n")
info("Data stored successfully")