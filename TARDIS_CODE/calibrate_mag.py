#!/usr/bin/env python3
"""
Calibração de Magnetômetro (Hard Iron + Soft Iron opcional)
Uso: Execute e gire o robô em todas as direções (figura de 8 ou movimentos aleatórios)
Até que a coleta termine. Os offsets serão salvos em 'mag_calib.json'.
"""

import time
import json
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Callable, Optional
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel   
navigator.init() # Inicia navigator
navigator.set_pwm_freq_hz(50)
navigator.set_pwm_enable(True)

# ============================================================
# 1. Interface com o sensor (substitua pela leitura real da Navigator)
# ============================================================
def read_magnetometer_raw() -> Tuple[float, float, float]:
    """
    Exemplo de leitura dos valores brutos do magnetômetro.
    Você DEVE substituir esta função pela leitura real da sua placa Navigator.
    Retorna uma tupla (x, y, z) em microteslas (uT) ou unidades cruas do sensor.
    """
    # Simulação (substitua pelo código real)
    # Exemplo com BlueRobotics Navigator (usando a biblioteca)
    # from bluerobotics_navigator import Magnetometer
    # mag = Magnetometer()
    # x, y, z = mag.read()
    # return x, y, z
    mag = navigator.read_mag()
    return mag.x, mag.y, mag.z
    # Placeholder para teste (gerando dados aleatórios)

# ============================================================
# 2. Coleta de dados
# ============================================================
def collect_mag_data(duration: float = 30.0,
                     callback: Optional[Callable[[int], None]] = None) -> np.ndarray:
    """
    Coleta dados do magnetômetro por 'duration' segundos.
    Retorna um array Nx3 com as leituras brutas.
    """
    print(f"Coletando dados do magnetômetro por {duration} segundos...")
    print("Gire o robô em todas as direções (figura de 8, rotações completas).")
    start_time = time.time()
    data = []
    last_print = start_time
    while time.time() - start_time < duration:
        x, y, z = read_magnetometer_raw()
        data.append([x, y, z])
        now = time.time()
        if now - last_print >= 1.0:
            remaining = duration - (now - start_time)
            print(f"  Tempo restante: {remaining:.1f} s  |  Pontos coletados: {len(data)}")
            last_print = now
        time.sleep(0.005)  # Hz, ajustável
    print(f"Coleta finalizada. {len(data)} amostras coletadas.")
    return np.array(data)

# ============================================================
# 3. Cálculo dos offsets (hard iron) e escalas (soft iron)
# ============================================================
def calculate_hard_iron_offsets(data: np.ndarray) -> Tuple[float, float, float]:
    """
    Calcula os offsets de hard iron como o ponto médio entre min e max de cada eixo.
    Retorna (offset_x, offset_y, offset_z).
    """
    min_vals = np.min(data, axis=0)
    max_vals = np.max(data, axis=0)
    offsets = (min_vals + max_vals) / 2.0
    print(f"Hard iron offsets calculados: X={offsets[0]:.2f}, Y={offsets[1]:.2f}, Z={offsets[2]:.2f}")
    return offsets[0], offsets[1], offsets[2]

def calculate_soft_iron_correction(data: np.ndarray, offsets: Tuple[float, float, float]) -> np.ndarray:
    """
    (Opcional) Calcula uma matriz de correção de soft iron (distorção de escala e rotação).
    Aplica a correção para que os dados fiquem o mais próximo possível de uma esfera.
    Retorna a matriz 3x3 de transformação.
    """
    # Remove hard iron
    ox, oy, oz = offsets
    corrected = data - np.array([ox, oy, oz])
    
    # Ajuste de elipsóide para esfera (método dos mínimos quadrados)
    # Implementação simplificada: calcula fatores de escala por eixo
    # Para calibração completa (incluindo cross-axis), use um algoritmo como ellipsoid_fit.
    # Aqui, uma versão simples:
    radii = np.std(corrected, axis=0)
    # Evita divisão por zero
    radii = np.maximum(radii, 1e-6)
    scaling = np.mean(radii) / radii
    soft_iron_matrix = np.diag(scaling)
    print(f"Soft iron scaling (diagonal): {scaling}")
    return soft_iron_matrix

# ============================================================
# 4. Função principal de calibração
# ============================================================
def calibrate_magnetometer(duration: float = 30.0,
                           do_soft_iron: bool = True,
                           save_file: str = "mag_calib.json"):
    """
    Executa a calibração completa: coleta, cálculo de hard iron e (opcional) soft iron.
    Salva os parâmetros em um arquivo JSON.
    """
    # Coleta
    raw_data = collect_mag_data(duration=duration)
    
    # Hard iron
    offset_x, offset_y, offset_z = calculate_hard_iron_offsets(raw_data)
    
    # Soft iron (opcional)
    soft_matrix = None
    if do_soft_iron:
        soft_matrix = calculate_soft_iron_correction(raw_data, (offset_x, offset_y, offset_z))
        # Converte matriz para lista para serialização JSON
        soft_matrix_list = soft_matrix.tolist()
    else:
        soft_matrix_list = None
    
    # Salva em JSON
    calib_data = {
        "hard_iron_offsets": [offset_x, offset_y, offset_z],
        "soft_iron_matrix": soft_matrix_list,
        "timestamp": time.time()
    }
    with open(save_file, "w") as f:
        json.dump(calib_data, f, indent=2)
    print(f"Parâmetros de calibração salvos em '{save_file}'")
    
    return calib_data

# ============================================================
# 5. Aplicação da calibração a uma nova leitura
# ============================================================
def apply_magnetometer_calibration(raw_x: float, raw_y: float, raw_z: float,
                                   calib_file: str = "mag_calib.json") -> Tuple[float, float, float]:
    """
    Carrega a calibração de um arquivo JSON e a aplica a uma leitura crua.
    Retorna os valores calibrados (x, y, z).
    """
    with open(calib_file, "r") as f:
        calib = json.load(f)
    ox, oy, oz = calib["hard_iron_offsets"]
    # Remove hard iron
    cx = raw_x - ox
    cy = raw_y - oy
    cz = raw_z - oz
    # Aplica soft iron, se disponível
    if calib["soft_iron_matrix"] is not None:
        soft = np.array(calib["soft_iron_matrix"])
        # Transforma vetor [cx, cy, cz] pela matriz soft
        calibrated = soft @ np.array([cx, cy, cz])
        return calibrated[0], calibrated[1], calibrated[2]
    else:
        return cx, cy, cz

# ============================================================
# 6. Exemplo de uso (execução principal)
# ============================================================
if __name__ == "__main__":
    # --- PASSO 1: Calibrar ---
    # Execute esta parte apenas uma vez, com o robô parado mas podendo ser movimentado.
    print("=== CALIBRAÇÃO DO MAGNETÔMETRO ===")
    print("Certifique-se de que o robô está longe de grandes interferências magnéticas.")
    print("Você deve girar o robô lentamente em todas as orientações (guinada, inclinação, rotação).")
    input("Pressione ENTER para iniciar a coleta de dados...")
    navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch2, 1.0)
    # Ajuste a duração conforme necessário (30 segundos é suficiente)
    calibrate_magnetometer(duration=60.0, do_soft_iron=True, save_file="mag_calib.json")
    
    print("\nCalibração concluída! Agora você pode usar apply_magnetometer_calibration() nas suas leituras.")
    
    # --- PASSO 2: Teste rápido (opcional) ---
    # Exemplo: lê 10 amostras calibradas
    print("\nTestando calibração (10 leituras):")
    for _ in range(50):
        raw = read_magnetometer_raw()
        cal = apply_magnetometer_calibration(*raw, "mag_calib.json")
        print(f"Raw: {raw} -> Calibrado: {cal}")
        time.sleep(0.1)