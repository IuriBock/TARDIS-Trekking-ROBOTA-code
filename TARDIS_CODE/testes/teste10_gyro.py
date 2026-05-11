import math
import bluerobotics_navigator as navigator
import time
from send_data_teste import enviar_dados
import sys

try:
    navigator.init()
    print("Navigator inicializado com sucesso.")
    time.sleep(0.5)
except Exception as e:
    print(f"Falha na inicialização: {e}")
    sys.exit(1)

class GyroYawEstimator:
    def __init__(self):
        # estado
        self.yaw = 0.0  # rad

        self.gyro_scale = 0.26#360/359.5 # fator de ajuste

        # bias do giroscópio
        self.gyro_bias_z = 0.011
        self.bias_alpha = 0.1  # aprendizado lento

        # controle de tempo
        self.initialized = False

        # threshold para detectar robô parado
        self.still_threshold = 0.6  # rad/s (ajuste!)

    def update(self, gyro_z, dt):
        """
        Atualiza o yaw usando apenas o giroscópio

        Args:
            gyro_z: velocidade angular em Z (rad/s)
            dt: tempo entre leituras (s)

        Returns:
            yaw estimado (rad) [0, 2π)
        """

        if dt <= 0:
            return self.yaw

        # 🧠 1. ESTIMATIVA DE BIAS (quando parado)
        if abs(gyro_z) < self.still_threshold:
            self.gyro_bias_z = (
                (1 - self.bias_alpha) * self.gyro_bias_z +
                self.bias_alpha * gyro_z
            )
        print(self.gyro_bias_z)
        # 🧠 2. REMOVE BIAS
        corrected_gyro = (gyro_z - self.gyro_bias_z) * self.gyro_scale
        # 2.1 Aplica filtragem simples (média com leitura anterior)
        if not self.initialized:
            self.prev_corrected_gyro = corrected_gyro
            self.initialized = True
        else:
            corrected_gyro = (corrected_gyro + self.prev_corrected_gyro) / 2
            self.prev_corrected_gyro = corrected_gyro

        # 🧠 3. INTEGRAÇÃO
        self.yaw -= corrected_gyro * dt

        # 🧠 4. NORMALIZAÇÃO [0, 2π)
        self.yaw = self.yaw % (2 * math.pi)

        return self.yaw

if __name__ == "__main__":
    # Teste simples do estimador de yaw
    estimator = GyroYawEstimator()
    t_ant = time.time()
    t0 = time.time()
    while True:
        t_now = time.time()
        dt = t_now - t_ant
        t_ant = t_now
        gyro_z = navigator.read_gyro().z  # rad/s
        yaw = estimator.update(gyro_z,dt)
        enviar_dados(f"yaw={yaw:.1f}")
        print(f"Yaw:{math.degrees(yaw):.4f}º | {yaw:.4f} rad | dt={dt:.3f}")
        time.sleep(0.01)

#357
#358.1
#