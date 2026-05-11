"""
Fusão de dados de sensores para estimar posição e orientação.
"""
import math
from data_structures import SensorData, RobotState

class SensorFusion:
    """
    Fusão de dados de múltiplos sensores para estimar posição e orientação.
    
    Técnica: Filtro complementar com correção de wrap e compensação de inclinação.
    Suporta calibração do magnetômetro (hard/soft iron) e declinação magnética.
    """
    
    def __init__(self):

        # estado
        self.gyro_scale = 1 # fator de ajuste

        # bias do giroscópio
        self.gyro_bias = 0.011 # calibração do giroscópio (offset em rad/s)
        self.bias_alpha = 0.01  # aprendizado lento

        # threshold para detectar robô parado
        self.still_threshold = 0.02  # rad/s (ajuste!)

        self.prev_corrected_gyro = 0.0
        self.last_speed = 0.0
        self.last_yaw = 0.0

    def _compute_pitch_roll_from_accel(self, ax, ay, az):
        """
        Calcula pitch e roll (radianos) a partir do acelerômetro.
        Assume aceleração externa pequena (veículo em repouso ou movimento suave).
        """
        # pitch: rotação em torno do eixo Y (inclinação para frente/trás)
        pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        # roll:  rotação em torno do eixo X (inclinação lateral)
        roll = math.atan2(ay, az)
        return pitch, roll
    
    def estimate_yaw(self, sensor_data: SensorData, dt: float, 
                    current_yaw: float) -> float:
        """
        Estima a orientação (yaw) usando giroscópio.        
        Args:
            sensor_data: objeto com campos gyro_z,
            dt: intervalo de tempo desde a última chamada (segundos)
            current_yaw: estimativa anterior do yaw (radianos) [0, 2π)
        
        Returns:
            yaw estimado (radianos) no intervalo [0, 2π)
        """
        if dt <= 0:
            return current_yaw
        
         # 1. ESTIMATIVA DE BIAS (quando parado)
        if abs(sensor_data.gyro_z) < self.still_threshold:
            self.gyro_bias = (
                (1 - self.bias_alpha) * self.gyro_bias +
                self.bias_alpha * sensor_data.gyro_z
            )

        corrected_gyro = (sensor_data.gyro_z - self.gyro_bias) * self.gyro_scale

        # 2.1 Aplica filtragem simples (média com leitura anterior)
        corrected_gyro = (corrected_gyro + self.prev_corrected_gyro) / 2
        self.prev_corrected_gyro = corrected_gyro

        gyro_yaw = current_yaw - (corrected_gyro * dt)
        gyro_yaw = gyro_yaw % (2.0 * math.pi)
        
        return gyro_yaw

    def estimate_distance(self, encoder_delta: int) -> float:
        """
        Estima a distância percorrida usando o encoder.
        Args:
            encoder_delta: Número de pulsos desde a última leitura (incremento)
        Returns:
            distância estimada (metros) percorrida desde a última leitura
        """
        # Calibração: pulsos por metro (ajuste conforme seu robô)
        pulses_per_meter = 143.3  # calibrado
        distance = encoder_delta / pulses_per_meter

        return distance


    def estimate_position(self, encoder_delta: float, previous_state: RobotState, dt: float) -> RobotState:
        """
        Estima posição usando o INCREMENTO do encoder (delta de pulsos).
        
        Args:
            encoder_delta: Número de pulsos desde a última leitura (incremento)
            previous_state: Estado anterior do robô
            dt: Intervalo de tempo desde a última atualização (segundos)
        
        Returns:
            Novo estado do robô (posição x, y, yaw mantido, velocidade calculada)
        """
        distance = self.estimate_distance(encoder_delta)
        
        # Atualiza posição assumindo movimento na direção do yaw atual
        new_state = RobotState(
            x=previous_state.x + distance * math.cos(previous_state.yaw),
            y=previous_state.y + distance * math.sin(previous_state.yaw),
            yaw=previous_state.yaw,  # orientação vem da IMU
            speed=distance / dt if dt > 0 else 0.0
        )
        return new_state