import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel
import math
import time

navigator.init()
time.sleep(0.5)
while True:
    mag = navigator.read_mag()
    gyro = navigator.read_gyro()

    gy_x = gyro.x
    gy_y = gyro.y
    gy_z = gyro.z

    mag_x = mag.x
    mag_y = mag.y
    mag_z = mag.z

    print(f"mag: ({mag_x:.4f}, {mag_y:.4f}, {mag_z:.4f}) ")
    print(f"gyro: ({gy_x:.4f}, {gy_y:.4f}, {gy_z:.4f}) ")
    time.sleep(0.2)

    