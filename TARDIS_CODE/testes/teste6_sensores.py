import bluerobotics_navigator as navigator
import time 

navigator.init()
print("Esperando sensor estabilizar...")
time.sleep(1)  # Espera o sensor estabilizar


def ler_acelerometro():
    accel = navigator.read_accel()
    ax = accel.x
    ay = accel.y
    az = accel.z 
    return (ax, ay, az)  # (ax, ay, az) em m/s²

def ler_giroscopio():
    gyro = navigator.read_gyro()
    gx = gyro.x
    gy = gyro.y
    gz = gyro.z
    return (gx, gy, gz)  # (gx, gy, gz) em rad/s

def ler_magnetometro():
    mag = navigator.read_mag()
    mx = mag.x 
    my = mag.y
    mz = mag.z
    return (mx, my, mz)  # (mx, my, mz) em µT



while True:
    time.sleep(0.3)
    print("Acelerômetro (m/s²): ", ler_acelerometro())
    print("Giroscópio (rad/s): ", ler_giroscopio())
    print("Magnetômetro (µT): ", ler_magnetometro())