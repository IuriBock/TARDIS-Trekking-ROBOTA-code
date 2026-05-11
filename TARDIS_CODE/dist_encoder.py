from robot_control.hardware_interface import ler_encoder
import time

while(True):
    ler_encoder()
    print(ler_encoder())
    time.sleep(1)
