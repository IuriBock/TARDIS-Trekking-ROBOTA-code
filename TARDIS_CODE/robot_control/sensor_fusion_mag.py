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
        # Coeficiente base para o filtro complementar (será ajustado dinamicamente)
        self.tau = 3.5                     # constante de tempo (segundos)
        self.alpha = self.tau / (self.tau + 0.01)                  # valor inicial (será recalculado)
        
        self.gyro_scale = 360/(360-32)
        self.gyro_bias = math.radians(0.61)                # calibração do giroscópio (offset em rad/s)

        self.mag_yaw_offset = None      # valor do offset inicial (rad)
        self.offset_calibrated = False  # flag para indicar que o offset foi definido

        # Declinação magnética local (convertida para radianos)
        self.mag_declination = math.radians(-22.5)  # ajuste conforme sua localização
        
        # Parâmetros de calibração do magnetômetro (hard e soft iron)
        # Inicialmente sem correção (offset zero, fator 1)
        #self.mag_offset = [ -29.5, 1.875, 0.0 ]   # µT
        #self.mag_offset = [-11.25, -19.95, 0.0]   # µT
        self.mag_offset = [0.0, 0.0, 0.0]   # µT

        self.mag_scale  = [1.0, 1.0, 1.0]   # adimensional
               

        # Limiares para validação do magnetômetro
        self.min_horizontal_field = 3.0     # µT (campo horizontal mínimo para bússola)
        self.expected_field_min = 10.0      # µT (campo total esperado mínimo)
        self.expected_field_max = 50.0      # µT (campo total esperado máximo)
        
        # Valores auxiliares para suavização opcional de pitch/roll
        self.last_pitch = 0.0
        self.last_roll = 0.0
        self.alpha_acc = 0.2                # suavização exponencial para pitch/roll
    
    def _compute_mag_yaw_raw(self, sensor_data: SensorData):
        """
        Calcula o yaw do magnetômetro (rad) sem aplicar offset, retorna None se leitura inválida.
        """
        # Calibração hard/soft iron
        mx, my, mz = self._calibrate_mag(sensor_data.mag_x,
                                        sensor_data.mag_y,
                                        sensor_data.mag_z)
        
        # Validação da intensidade total
        total_field = math.sqrt(mx*mx + my*my + mz*mz)
        if not (self.expected_field_min <= total_field <= self.expected_field_max):
            return None
        
        # Compensação de tilt se acelerômetro disponível
        use_accel = hasattr(sensor_data, 'accel_x') and sensor_data.accel_x is not None
        if use_accel:
            pitch, roll = self._compute_pitch_roll_from_accel(sensor_data.accel_x,
                                                            sensor_data.accel_y,
                                                            sensor_data.accel_z)
            # Suavização opcional (pode ser mantida ou removida)
            pitch = self.alpha_acc * pitch + (1.0 - self.alpha_acc) * self.last_pitch
            roll  = self.alpha_acc * roll  + (1.0 - self.alpha_acc) * self.last_roll
            self.last_pitch, self.last_roll = pitch, roll
            
            Xh = mx * math.cos(roll) + mz * math.sin(roll)
            Yh = (mx * math.sin(pitch) * math.sin(roll) +
                my * math.cos(pitch) -
                mz * math.sin(pitch) * math.cos(roll))
        else:
            Xh = mx
            Yh = my
        
        horizontal_norm = math.hypot(Xh, Yh)
        if horizontal_norm <= self.min_horizontal_field:
            return None
        
        mag_yaw = math.atan2(Yh, Xh)
        mag_yaw -= self.mag_declination      # correção de declinação
        return mag_yaw % (2.0 * math.pi)
    
    def calibrate_initial_heading(self, get_sensor_data_func, num_samples=100, delay=0.01):
        """
        Coleta várias leituras do magnetômetro, calcula a média circular e define como offset.
        A orientação atual do robô passará a ser considerada yaw = 0.
        """
        import time
        yaws = []
        for _ in range(num_samples):
            data = get_sensor_data_func()
            mag_yaw = self._compute_mag_yaw_raw(data)
            if mag_yaw is not None:
                yaws.append(mag_yaw)
            time.sleep(delay)
        
        if yaws:
            # Média circular para evitar problemas com wrap em 0/360°
            avg_sin = sum(math.sin(y) for y in yaws) / len(yaws)
            avg_cos = sum(math.cos(y) for y in yaws) / len(yaws)
            self.mag_yaw_offset = math.atan2(avg_sin, avg_cos)
            self.offset_calibrated = True
            print(f"[Fusion] Offset inicial do magnetômetro: {math.degrees(self.mag_yaw_offset):.1f}°")
            return True
        else:
            print("[Fusion] Falha ao calibrar offset inicial (leituras inválidas).")
            return False 
        
    def load_mag_calibration(self, filepath: str = "mag_calib.json"):
        """
        Carrega os parâmetros de calibração do magnetômetro a partir de um arquivo JSON.
        Espera o formato gerado por calibrate_mag.py:
            {
                "hard_iron_offsets": [ox, oy, oz],
                "soft_iron_matrix": [[sxx, sxy, sxz], ...]  # matriz 3x3
            }
        Atualiza self.mag_offset e self.mag_scale (diagonal da matriz soft iron).
        """
        import json
        try:
            with open(filepath, 'r') as f:
                calib = json.load(f)
            
            # Hard iron offsets
            offsets = calib.get("hard_iron_offsets")
            if offsets and len(offsets) == 3:
                self.mag_offset = offsets[:]
                print(f"[Fusion] Offsets carregados: X={offsets[0]:.2f}, Y={offsets[1]:.2f}, Z={offsets[2]:.2f}")
            
            # Soft iron matrix (assumimos diagonal para escala)
            soft_matrix = calib.get("soft_iron_matrix")
            if soft_matrix and len(soft_matrix) == 3:
                # Extrai os elementos diagonais como fatores de escala
                sx = soft_matrix[0][0]
                sy = soft_matrix[1][1]
                sz = soft_matrix[2][2]
                self.mag_scale = [sx, sy, sz]
                print(f"[Fusion] Escalas carregadas: X={sx:.4f}, Y={sy:.4f}, Z={sz:.4f}")
            else:
                print("[Fusion] Sem soft iron matrix, mantendo escalas padrão.")
        except FileNotFoundError:
            print(f"[Fusion] Arquivo {filepath} não encontrado. Usando calibração padrão.")
        except Exception as e:
            print(f"[Fusion] Erro ao carregar calibração: {e}")
    def calibrate_magnetometer(self, offsets, scales):
        """
        Define os offsets (hard iron) e fatores de escala (soft iron) do magnetômetro.
        
        Args:
            offsets: lista [offset_x, offset_y, offset_z] em µT
            scales:  lista [scale_x, scale_y, scale_z] (adimensional)
        """
        self.mag_offset = offsets[:]
        self.mag_scale  = scales[:]
    
    def set_mag_declination(self, declination_degrees):
        """
        Define a declinação magnética local em graus (positiva para leste).
        """
        self.mag_declination = math.radians(declination_degrees)
    
    def _calibrate_mag(self, raw_x, raw_y, raw_z):
        """Aplica correção de hard e soft iron às leituras brutas."""
        x = (raw_x - self.mag_offset[0]) * self.mag_scale[0]
        y = (raw_y - self.mag_offset[1]) * self.mag_scale[1]
        z = (raw_z - self.mag_offset[2]) * self.mag_scale[2]
        return x, y, z
    
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
        Estima a orientação (yaw) usando giroscópio e magnetômetro.
        Opcionalmente usa acelerômetro para compensar a inclinação (tilt).
        
        Args:
            sensor_data: objeto com campos gyro_z, mag_x, mag_y, mag_z,
                         e opcionalmente acc_x, acc_y, acc_z (m/s²)
            dt: intervalo de tempo desde a última chamada (segundos)
            current_yaw: estimativa anterior do yaw (radianos) [0, 2π)
        
        Returns:
            yaw estimado (radianos) no intervalo [0, 2π)
        """
        # 1. Integração do giroscópio
        corrected_gyro = (sensor_data.gyro_z - self.gyro_bias) * self.gyro_scale
        gyro_yaw = current_yaw - corrected_gyro * dt
        gyro_yaw = gyro_yaw % (2.0 * math.pi)
        
        # 2. Obter yaw do magnetômetro bruto (se válido)
        mag_yaw_raw = self._compute_mag_yaw_raw(sensor_data)
        if mag_yaw_raw is not None:
            mag_yaw = mag_yaw_raw
        else:
            mag_yaw = gyro_yaw   # fallback para o giroscópio
        
        # 3. Aplicar offset relativo (se já calibrado)
        if self.offset_calibrated and self.mag_yaw_offset is not None:
            mag_yaw = (mag_yaw - self.mag_yaw_offset) % (2.0 * math.pi)

        # 4. Fusão complementar (igual ao original)
        error = mag_yaw - gyro_yaw
        error = math.atan2(math.sin(error), math.cos(error))
        
        if dt > 0:
            self.alpha = self.tau / (self.tau + dt)
        else:
            self.alpha = 1.0
        fused_yaw = gyro_yaw + (1.0 - self.alpha) * error
        fused_yaw = fused_yaw % (2.0 * math.pi)
        return fused_yaw
    
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
        # Calibração: pulsos por metro (ajuste conforme seu robô)
        pulses_per_meter = 143.3  # ← ADAPTAR
        
        # Distância percorrida neste intervalo
        distance = encoder_delta / pulses_per_meter
        
        # Atualiza posição assumindo movimento na direção do yaw atual
        new_state = RobotState(
            x=previous_state.x + distance * math.cos(previous_state.yaw),
            y=previous_state.y + distance * math.sin(previous_state.yaw),
            yaw=previous_state.yaw,  # orientação vem da IMU
            speed=distance / dt if dt > 0 else 0.0
        )
        return new_state