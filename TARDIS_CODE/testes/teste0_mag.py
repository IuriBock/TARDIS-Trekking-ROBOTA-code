import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel
import math
import time
navigator.init()
time.sleep(0.5)
while True:
    mag = navigator.read_mag()
    gyro = navigator.read_gyro()
    gyro_z = gyro.z
    mag_x = mag.x
    mag_y = mag.y
    mag_z = mag.z
    
    threshold_gyro_still = 0.1  # rad/s (ajuste conforme seu giroscópio)
    if abs(gyro_z) < threshold_gyro_still:
        gyro_z = 0.0  # Considera o robô parado para o cálculo do bias
    mag_raw_yaw = math.atan2(mag_y, mag_x)        # radianos
    mag_raw_yaw_deg = math.degrees(mag_raw_yaw)   # -180 a 180
    print(f"mag: ({mag_x:.2f}, {mag_y:.2f}, {mag_z:.2f}) ")
    #print(f"gyro_z: {gyro_z:.5f}")
    time.sleep(0.3)