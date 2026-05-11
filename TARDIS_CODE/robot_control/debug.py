# teste_sensores.py
from hardware_interface import ler_todos_sensores
import time

print("Testando ler_todos_sensores()...")
for i in range(10):
    try:
        dados = ler_todos_sensores()
        print(f"Iteração {i}: encoder={dados.encoder_rear}, accel=({dados.accel_x:.2f}, {dados.accel_y:.2f}, {dados.accel_z:.2f})")
    except Exception as e:
        print(f"ERRO: {e}")
        import traceback
        traceback.print_exc()
    time.sleep(0.5)