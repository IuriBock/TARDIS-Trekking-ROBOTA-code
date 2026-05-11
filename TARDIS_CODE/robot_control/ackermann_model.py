"""
Modelo cinemático de direção Ackermann.
"""
import math
from typing import Tuple
from data_structures import RobotState


class AckermannModel:
    """
    Modelo de direção Ackermann para robôs com rodas dianteiras direcionais.
    
    ADAPTAÇÕES NECESSÁRIAS:
    1. Definir as dimensões do seu robô (distância entre eixos, bitola)
    2. Ajustar limites de ângulo de direção
    3. Ajustar relação entre comando de direção e ângulo real
    """
    
    def __init__(self):
        # DIMENSÕES DO ROBÔ - ADAPTAR CONFORME SEU ROBÔ
        self.wheelbase = 0.3  # Distância entre eixos dianteiro e traseiro (metros)
        self.track_width = 0.265  # Bitola do veículo (metros)

        # Ângulos máximos (em radianos)
        self.max_steering_left = math.radians(35)   # para esquerda (positivo)
        self.max_steering_right = math.radians(25)  # para direita (negativo)

        # Relação de transmissão direção (se houver)
        self.steering_ratio = 1.0  # 1:1 por padrão
        
    def limit_steering_angle(self, steering_angle: float) -> float:
        """
        Limita o ângulo de direção respeitando os limites assimétricos.
        
        Args:
            steering_angle: Ângulo desejado (radianos, positivo = esquerda)
            
        Returns:
            Ângulo limitado (radianos)
        """
        # Positivo = esquerda, Negativo = direita
        if steering_angle > 0:  # Esquerda
            return min(steering_angle, self.max_steering_left)
        else:  # Direita
            return max(steering_angle, -self.max_steering_right)
    
    def calculate_curvature(self, steering_angle: float) -> float:
        """
        Calcula a curvatura baseado no ângulo de direção limitado.
        """
        # Limita o ângulo de direção
        steering_angle = self.limit_steering_angle(steering_angle)
        
        # Evita divisão por zero
        if abs(steering_angle) < 0.001:
            return 0.0
        
        # Cálculo da curvatura
        curvature = math.tan(steering_angle) / self.wheelbase
        
        return curvature
    
    def calculate_wheel_angles(self, steering_angle: float) -> Tuple[float, float]:
        """
        Calcula os ângulos das rodas dianteiras para movimento Ackermann puro.
        Considera os limites assimétricos.
        """
        # Limita o ângulo de direção
        steering_angle = self.limit_steering_angle(steering_angle)
        
        # Para simplificação, usamos o mesmo ângulo para ambas as rodas
        # Em uma implementação mais avançada, calcular os ângulos ideais
        return steering_angle, steering_angle
    
    def update_odometry(self, state: RobotState, speed: float, 
                       steering_angle: float, dt: float) -> RobotState:
        """
        Atualiza a odometria usando ângulo limitado.
        """
        # Limita o ângulo de direção
        steering_angle = self.limit_steering_angle(steering_angle)
        
        # Cálculo da curvatura
        if abs(steering_angle) < 0.001:
            # Movimento retilíneo
            dx = speed * math.cos(state.yaw) * dt
            dy = speed * math.sin(state.yaw) * dt
            dyaw = 0.0
        else:
            # Movimento curvilíneo
            curvature = math.tan(steering_angle) / self.wheelbase
            radius = 1.0 / curvature if abs(curvature) > 0.001 else float('inf')
            
            # Velocidade angular
            omega = speed * curvature
            
            # Atualização da posição e orientação
            if abs(radius) < 100:
                dx = (math.sin(state.yaw + omega * dt) - math.sin(state.yaw)) * radius
                dy = (math.cos(state.yaw) - math.cos(state.yaw + omega * dt)) * radius
            else:
                dx = speed * math.cos(state.yaw) * dt
                dy = speed * math.sin(state.yaw) * dt
            
            dyaw = omega * dt
        
        # Atualiza o estado
        new_state = RobotState(
            x=state.x + dx,
            y=state.y + dy,
            yaw=(state.yaw + dyaw) % (2 * math.pi),
            speed=speed,
            steering_angle=steering_angle,
            acceleration=(speed - state.speed) / dt if dt > 0 else 0.0,
            yaw_rate=dyaw / dt if dt > 0 else 0.0
        )
        
        return new_state