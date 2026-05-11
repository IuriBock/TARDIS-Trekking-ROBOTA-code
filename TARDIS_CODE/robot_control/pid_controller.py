"""
Controlador PID genérico com anti-windup e reset do integral.
"""
import time
from typing import Tuple

class PIDController:
    """Controlador PID com anti-windup e reset automático"""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limits: Tuple[float, float] = (-1.0, 1.0),
                 sample_time: float = 0.01,
                 integral_limit: float = 1.0):
        """
        Args:
            kp, ki, kd: Ganhos proporcional, integral e derivativo
            output_limits: Limites mínimo e máximo da saída
            sample_time: Tempo mínimo entre atualizações (segundos)
            integral_limit: Limite máximo do termo integral (anti-windup)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output, self.max_output = output_limits
        self.sample_time = sample_time
        self.integral_limit = integral_limit
        
        self.reset()
    
    def reset(self):
        """Reseta completamente o controlador"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
        self.last_output = 0.0
    
    def reset_integral(self):
        """Reseta apenas o termo integral (útil após atingir o alvo)"""
        self.integral = 0.0
    
    def compute(self, setpoint: float, measured_value: float, 
                reset_integral: bool = False) -> float:
        """
        Calcula a saída do controlador PID.
        
        Args:
            setpoint: Valor desejado
            measured_value: Valor medido
            reset_integral: Se True, reseta o termo integral antes do cálculo
        """
        current_time = time.time()
        dt = current_time - self.prev_time
        
        # Reseta integral se solicitado
        if reset_integral:
            self.integral = 0.0
        
        if dt < self.sample_time:
            return self.last_output
        
        # Erro atual
        error = setpoint - measured_value
        
        # Termo proporcional
        p_term = self.kp * error
        
        # Termo integral com anti-windup
        if abs(error) < 0.1:  # Só acumula erro quando está próximo do alvo
            self.integral += error * dt
            # Limita o termo integral
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        
        i_term = self.ki * self.integral
        
        # Termo derivativo
        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        
        # Soma dos termos
        output = p_term + i_term + d_term
        
        # Saturação da saída com anti-windup
        if output > self.max_output:
            output = self.max_output
            # Congela o integral quando saturado (anti-windup)
            if error * output > 0:
                self.integral -= error * dt
        elif output < self.min_output:
            output = self.min_output
            if error * output > 0:
                self.integral -= error * dt
        
        # Atualiza estados
        self.prev_error = error
        self.prev_time = current_time
        self.last_output = output
        
        return output