from hardware_interface import inicializar_hardware, ler_encoder
import time

inicializar_hardware()

while(True):
    ler_encoder()
    print(ler_encoder())
    time.sleep(1)
