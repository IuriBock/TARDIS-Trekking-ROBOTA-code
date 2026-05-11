import bluerobotics_navigator as navigator
import time

navigator.init()
print("Aguardando sensor estabilizar...")
time.sleep(2)  # Espera 2 segundos

while True:
    time.sleep(0.1)
    try:
        mag = navigator.read_mag()
        print(f"Leitura única: {mag.x}, {mag.y}, {mag.z}")
    except Exception as e:
        print(f"Erro na leitura única: {e}")