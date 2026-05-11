"""
Estruturas de dados compartilhadas entre os módulos.
"""
from dataclasses import dataclass

@dataclass
class SensorData:
    """Estrutura para armazenar dados dos sensores"""
    # Acelerômetro (m/s²)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    
    # Giroscópio (rad/s ou graus/s - verifique seu sensor)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    
    # Magnetômetro (microTesla ou Gauss - verifique seu sensor)
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    
    # Encoders (rotações ou pulsos)
    encoder_rear: int = 0  
    
    # Timestamp
    timestamp: float = 0.0

@dataclass
class RobotState:
    """Estado atual do robô"""
    x: float = 0.0          # Posição X (metros)
    y: float = 0.0          # Posição Y (metros)
    yaw: float = 0.0        # Orientação (radianos)
    speed: float = 0.0      # Velocidade linear (m/s)
    steering_angle: float = 0.0  # Ângulo de direção (radianos)
    
    # Derivadas (opcionais)
    acceleration: float = 0.0
    yaw_rate: float = 0.0

@dataclass
class TrajectoryPoint:
    """Ponto na trajetória predefinida"""
    x: float
    y: float
    speed: float  # Velocidade desejada neste ponto