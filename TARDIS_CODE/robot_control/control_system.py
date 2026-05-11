"""
Sistema principal de controle para robô com direção Ackermann.
"""
import time
import math
import numpy as np
import threading
from queue import Queue, Full, Empty
from typing import List, Optional
import sys
import os

from send_data import enviar_dados
from data_structures import SensorData, RobotState, TrajectoryPoint
from pid_controller import PIDController
from ackermann_model import AckermannModel
from sensor_fusion import SensorFusion

import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel, UserLed

# Adiciona o diretório atual ao path para importar o servidor
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

class AckermannControlSystem:
    """
    Sistema principal de controle para robô com direção Ackermann.
    
    Este sistema:
    1. Lê dados dos sensores (adaptar para sua interface de hardware)
    2. Faz fusão dos dados para estimar estado
    3. Calcula trajetória e controladores PID
    4. Gera comandos para atuadores
    """
    
    def __init__(self, simulation_mode: bool=False, web_server: bool=True):
        """
        Inicializa o sistema de controle.
        Args:
            simulation_mode: Se True, usa modelo cinemático em vez de sensores reais
        """
        self.motor_pin = PwmChannel.Ch2 # Canal PWM para o motor (ajuste conforme seu hardware)
        self.servo_pin = PwmChannel.Ch1  # Canal PWM para o servo de direção (ajuste conforme seu hardware)
        self.led_pin = PwmChannel.Ch3    # Canal PWM para LED de emergência (opcional)

        self.led_b = UserLed.Led2
        self.led_r = UserLed.Led3
        self.led_g = UserLed.Led1

        # Modo de operação
        self.simulation_mode = simulation_mode
        
        # Modelo do robô
        self.ackermann_model = AckermannModel()
        
        # Fusão de sensores
        self.sensor_fusion = SensorFusion()
        
        # Estado atual do robô
        self.current_state = RobotState()

        # Último valor do encoder (para calcular delta)
        self.last_encoder_value = 0
        
        # Trajetória a ser seguida
        self.trajectory: List[TrajectoryPoint] = []
        self.current_waypoint_index = 0

        # Velocidade padrão (m/s) caso o ponto não especifique
        self.cruise_speed = 5.0   # ajuste conforme seu robô

        # Distância para começar a desacelerar (metros)
        self.deceleration_distance = 5.0   # começa a reduzir velocidade a 0.5m do alvo

        # Velocidade mínima na aproximação (m/s)
        self.min_approach_speed = 0.01

        # Distância para parar completamente (deve ser menor que waypoint_tolerance)
        self.stop_distance = 0.03
# ========================================================================================
# CONSTANTES PARA AJUSTE

        # Controladores PID - ADAPTAR GANHOS
        self.pid_speed = PIDController(
            kp=0.800, ki=0.020, kd=0.010,
            output_limits=(0.0, 1.0),  # 0% a 100% de potência
            integral_limit=0.5          # Limite do termo integral
        )

        self.pid_steering = PIDController( # kp=2.00, ki=0.040, kd=0.120,
            kp=2.5, ki=0.04, kd=0.05,
            output_limits=(-1.0, 1.0),  # Limites do ângulo de direção
            integral_limit=0.3          # Limite do termo integral
        )
        
        self.pid_lateral = PIDController(
            kp=2.5, ki=0.03, kd=0.1,
            output_limits=(-0.3, 0.3)  # Correção lateral
        )
        
        # Configurações
        self.lookahead_distance = 0.3  # Distância de antecipação (metros)
        self.waypoint_tolerance = 0.1  # Tolerância para considerar waypoint alcançado

# ========================================================================================     

        # Thread de controle
        self.control_thread = None
        self.running = False
        self.sensor_queue = Queue(maxsize=1)

        # Simulação: tempo e logs
        self.sim_time = 0.0
        self.log_file = None
        
        self.DUTY_MIN_MOTOR = 0.5  # Ajuste conforme calibração
        self.DUTY_MAX_MOTOR = 1.0

        if simulation_mode:
            print("[SIMULAÇÃO] Modo ativado - robô virtual")
    def load_trajectory(self, trajectory_points):
        """
        Carrega trajetória a partir de pontos.
        Formatos aceitos:
            - Lista de tuplas (x, y) -> velocidade padrão
            - Lista de tuplas (x, y, speed) -> velocidade definida por ponto
            - Lista de objetos TrajectoryPoint
        """
        self.trajectory = []
        for pt in trajectory_points:
            if isinstance(pt, TrajectoryPoint):
                self.trajectory.append(pt)
            elif isinstance(pt, (list, tuple)):
                if len(pt) == 2:
                    x, y = pt
                    speed = self.cruise_speed
                elif len(pt) >= 3:
                    x, y, speed = pt[0], pt[1], pt[2]
                else:
                    continue
                self.trajectory.append(TrajectoryPoint(x, y, speed=speed))
            else:
                raise ValueError(f"Formato inválido: {pt}")
        
        self.current_waypoint_index = 0
        print(f"[Trajetória] Carregados {len(self.trajectory)} pontos")
    
    def calculate_steering_control(self, target_point: TrajectoryPoint) -> float:
        """
        Calcula o controle de direção usando erro de orientação e lateral.
        """
        # 1. Calcula o ângulo desejado para o ponto alvo
        dx = target_point.x - self.current_state.x
        dy = target_point.y - self.current_state.y
        target_angle = math.atan2(dy, dx)
        
        # 2. Erro de orientação (diferença entre ângulo atual e desejado)
        yaw_error = target_angle - self.current_state.yaw
        # Normaliza para [-π, π]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        
        # 3. Erro lateral (distância perpendicular à trajetória)
        # Vetor direção atual
        current_dir = np.array([math.cos(self.current_state.yaw), 
                               math.sin(self.current_state.yaw)])
        # Vetor para o alvo
        target_vec = np.array([dx, dy])
        distance_to_target = np.linalg.norm(target_vec)
        
        if distance_to_target > 0.01:
            target_vec = target_vec / distance_to_target
            # Produto cruzado (erro lateral)
            lateral_error = np.cross(current_dir, target_vec)
        else:
            lateral_error = 0.0
        
        # 4. Combina os erros (prioriza orientação perto do alvo)
        if distance_to_target > 0.5:
            # Longe do alvo: prioriza orientação
            combined_error = yaw_error * 0.7 + lateral_error * 0.3
        else:
            # Perto do alvo: prioriza correção lateral
            combined_error = yaw_error * 0.3 + lateral_error * 0.7
        
        # Limita o erro para evitar comandos extremos
        combined_error = max(-1.0, min(1.0, combined_error))
        
        # 5. Reseta integral se o erro for muito pequeno
        reset_integral = abs(combined_error) < 0.1
        
        # 6. Calcula o comando PID
        steering_cmd = self.pid_steering.compute(0.0, -combined_error, 
                                                  reset_integral=reset_integral)
        
        return steering_cmd
    
    def calculate_speed_control(self, target_point: TrajectoryPoint) -> float:
        """
        Controle de velocidade simplificado:
        - Velocidade constante (cruise_speed) quando longe do ponto.
        - Desaceleração linear (opcional) ao se aproximar.
        - Para completamente quando dentro de stop_distance.
        """
        dx = target_point.x - self.current_state.x
        dy = target_point.y - self.current_state.y
        distance = math.hypot(dx, dy)
        
        # Parada total se muito perto
        if distance < self.stop_distance:
            return 0.0
        
        # Desaceleração proporcional (suave) na zona de aproximação
        if distance < self.deceleration_distance:
            # Mapeia distance [stop_distance, deceleration_distance] -> [0, cruise_speed]
            # Evita divisão por zero
            range_len = self.deceleration_distance - self.stop_distance
            if range_len > 0:
                ratio = (distance - self.stop_distance) / range_len
                speed = ratio * self.cruise_speed
                # Garante que não ultrapasse cruise_speed e não fique negativo
                return max(0.0, min(speed, self.cruise_speed))
            else:
                return 0.0
        
        # Longe: velocidade máxima constante
        return self.cruise_speed
    
    def get_lookahead_point(self) -> Optional[TrajectoryPoint]:
        if not self.trajectory or self.current_waypoint_index >= len(self.trajectory):
            return None
        
        target = self.trajectory[self.current_waypoint_index]
        dx = target.x - self.current_state.x
        dy = target.y - self.current_state.y
        distance = math.hypot(dx, dy)
        
        if distance < self.waypoint_tolerance:
            print(f"[Waypoint] Ponto {self.current_waypoint_index} alcançado!")
            self.pid_speed.reset_integral()
            self.pid_steering.reset_integral()
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.trajectory):
                return None
            target = self.trajectory[self.current_waypoint_index]
        
        return target
        
    def update_sensors(self, sensor_data: SensorData):
        """
        Atualiza dados dos sensores e estima estado do robô.
        
        ADAPTAÇÃO: Integrar com sua interface de hardware real
        """
        if self.simulation_mode:
            # Em simulação, não precisamos de sensores reais
            # Mas podemos criar dados simulados para testar a fusão
            pass
        else:
            # Coloca dados na fila para processamento thread-safe
            #self.sensor_queue.put(sensor_data)
            # Garante que a fila tenha apenas o último dado
            try:
                self.sensor_queue.put_nowait(sensor_data)
            except Full:
                # Remove o item antigo e insere o novo
                try:
                    self.sensor_queue.get_nowait()
                except Empty:
                    pass
                self.sensor_queue.put_nowait(sensor_data)
            
    def process_sensors(self):
        """Processa dados dos sensores da fila"""
        last_time = time.time()
        self.last_encoder_value = 0  # inicializa com o primeiro valor lido

        while self.running:
            try:
                # Obtém dados dos sensores (não-bloqueante)
                sensor_data = self.sensor_queue.get(timeout=0.01)
                
                # Calcula dt
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                
                 # ===== CALCULA O DELTA DO ENCODER =====
                current_encoder = sensor_data.encoder_rear
                delta_encoder = current_encoder - self.last_encoder_value
                self.last_encoder_value = current_encoder  # atualiza para a próxima

                # Estima orientação usando IMU
                estimated_yaw = self.sensor_fusion.estimate_yaw(
                    sensor_data, dt, self.current_state.yaw
                )
                
                # Estima posição usando odometria
                estimated_state = self.sensor_fusion.estimate_position(
                    delta_encoder, self.current_state, dt
                )
                
                # Atualiza estado atual (fusão de estimativas)
                self.current_state.yaw = estimated_yaw
                self.current_state.x = estimated_state.x
                self.current_state.y = estimated_state.y
                self.current_state.speed = estimated_state.speed
                # Normaliza yaw após cada atualização
                self.current_state.yaw = self.current_state.yaw % (2 * math.pi)
                
            except:
                # Fila vazia ou outro erro - continua execução
                pass
    
    def control_loop(self):
        """Loop principal de controle"""
        print("Iniciando sistema de controle do robô...")
        
        # Thread para processamento de sensores
        sensor_thread = threading.Thread(target=self.process_sensors)
        sensor_thread.daemon = True
        sensor_thread.start()
        
        last_control_time = time.time()
        
        while self.running:
            try:
                # Taxa de atualização do controle (100 Hz)
                current_time = time.time()
                dt = current_time - last_control_time
                
                if dt < 0.01:  # 10 ms
                    time.sleep(0.01 - dt)
                    continue
                
                # Obtém ponto alvo na trajetória
                target_point = self.get_lookahead_point()
                
                if target_point is None:
                    print("Trajetória concluída ou vazia")
                    self.stop()
                    break
                
                # Cálculo dos comandos de controle
                steering_cmd = self.calculate_steering_control(target_point)
                speed_cmd = self.calculate_speed_control(target_point)
                
                # Aplica os comandos aos atuadores
                self.apply_control(steering_cmd, speed_cmd)

                # Log de estado (pode ser reduzido para evitar sobrecarga)
                freq = 10  # Frequência de log (Hz)
                if int(current_time * freq * 10) % 10 == 0:
                    enviar_dados(f"[Posição Atual]: x={self.current_state.x:.3f}| y={self.current_state.y:.3f}| yaw={self.current_state.yaw:.3f}")
                    #print("[Dados enviados]")

                last_control_time = current_time
                
            except Exception as e:
                print(f"Erro no loop de controle: {e}")
                self.stop()
                break
    
    def apply_control(self, steering_cmd: float, speed_cmd: float):
        """
        Aplica comandos de controle aos atuadores usando duty cycle.
        """
        # ===== 1. CONVERTE COMANDO PARA ÂNGULO FÍSICO =====
        if steering_cmd >= 0:
            steering_angle = steering_cmd * self.ackermann_model.max_steering_left
        else:
            steering_angle = steering_cmd * self.ackermann_model.max_steering_right
        
        # ===== 2. CONVERTE ÂNGULO PARA DUTY CYCLE DO SERVO =====
        # Valores de duty cycle do seu servo - VOCÊ PRECISA CALIBRAR ESTES 3 VALORES
        DUTY_CENTRO = 0.1048    # rodas retas (ajuste conforme seu servo)
        DUTY_ESQUERDA = 0.08   # giro máximo à esquerda
        DUTY_DIREITA = 0.12    # giro máximo à direita
        
        if steering_angle <= 0:  # Esquerda
            ratio = steering_angle / self.ackermann_model.max_steering_left
            duty = DUTY_CENTRO + ratio * (DUTY_ESQUERDA - DUTY_CENTRO)
        else:  # Direita
            ratio = -steering_angle / self.ackermann_model.max_steering_right
            duty = DUTY_CENTRO + ratio * (DUTY_DIREITA - DUTY_CENTRO)
        
        # ===== 3. CONVERTE VELOCIDADE PARA DUTY CYCLE DO MOTOR =====
        #DUTY_MAX_MOTOR = 1.0  # 100%
        
         # ===== MAPEAMENTO DA ZONA MORTA DO MOTOR =====
        # speed_cmd está em [-1, 1], vindo do PID de velocidade
        if abs(speed_cmd) < 0.02:          # Zona morta de comando (histerese)
            motor_duty = 0.0
            direcao = "parado"
        else:
            if speed_cmd > 0:
                # Mapeamento linear: [0,1] -> [D_min, 1]
                motor_duty = self.DUTY_MIN_MOTOR + speed_cmd * (1.0 - self.DUTY_MIN_MOTOR)
                direcao = "frente"
            else:  # speed_cmd < 0
                cmd_abs = abs(speed_cmd)
                motor_duty = - (self.DUTY_MIN_MOTOR + cmd_abs * (1.0 - self.DUTY_MIN_MOTOR))
                direcao = "ré"

            # Garantia de limites (segurança)
            motor_duty = max(-self.DUTY_MAX_MOTOR, min(self.DUTY_MAX_MOTOR, motor_duty))
            
        if self.simulation_mode:
            # ===== MODO SIMULAÇÃO =====
            # Atualiza o estado usando o modelo cinemático
            dt = 0.01  # Mesmo dt do control_loop
            self.current_state = self.ackermann_model.update_odometry(
                self.current_state,
                motor_duty * 0.7,  # Velocidade máxima simulada:  m/s
                steering_angle,
                dt
            )
            
            # Simula o encoder (para manter compatibilidade com o resto do código)
            # A cada 10ms, se moveu 0.005m, gera ~5 pulsos (assumindo 1000 pulsos/m)
            if hasattr(self, 'sim_encoder'):
                self.sim_encoder += int(abs(motor_duty) * 5)
            else:
                self.sim_encoder = 0
            
            # Debug visual (opcional)
            if int(self.sim_time * 10) % 10 == 0:
                enviar_dados(f"[Comando]: v={motor_duty*100:.2f}, steering={steering_angle:.2f}, modo=auto")
                print(f"[SIM] t={self.sim_time:.2f}s | Pos: ({self.current_state.x:.2f}, {self.current_state.y:.2f}) | "
                      f"Yaw: {math.degrees(self.current_state.yaw):.1f}° | "
                      f"Vel: {self.current_state.speed:.2f}m/s | "
                      f"Comando: motor={motor_duty:.2f}, dir={math.degrees(steering_angle):.1f}°")
            
            self.sim_time += dt

        else: # ===== MODO REAL =====
            # ===== 4. APLICA AOS ATUADORES =====
            # ADAPTAÇÃO: Substitua pelos comandos reais do seu hardware
            
            # Exemplo para servo com duty cycle (Raspberry Pi com RPi.GPIO):
            # servo_pwm.ChangeDutyCycle(duty * 100)  # converte para porcentagem
            navigator.set_pwm_channel_duty_cycle(self.servo_pin, duty)
            navigator.set_pwm_channel_duty_cycle(self.motor_pin, motor_duty)
            pass

        # Debug
        
        enviar_dados(f"[Comando]: v={motor_duty*100:.2f}, steering={steering_angle:.2f}, modo=auto")
        '''
        print(f"[COMANDOS] Ângulo: {math.degrees(steering_angle):.1f}° -> Duty={duty*100:.1f}% | "
            f"Velocidade: {speed_cmd*100:.1f}% -> Duty={motor_duty*100:.1f}%, {direcao}")
        '''
    def start(self):
        """Inicia o sistema de controle"""
        if self.running:
            print("Sistema já está em execução")
            return
        
        if not self.trajectory:
            print("Erro: Trajetória não definida")
            return
        
        self.running = True
        navigator.set_pwm_channel_duty_cycle(self.led_pin, 0.0)
        navigator.set_led(self.led_r, False)
        navigator.set_led(self.led_g, True)
        navigator.set_led(self.led_b, False)

        # Reseta controladores
        self.pid_speed.reset()
        self.pid_steering.reset()
        self.pid_lateral.reset()
        
        # Inicia thread de controle
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print("Sistema de controle iniciado")
    
    def stop(self):
        """Para o sistema de controle"""
        self.running = False

        navigator.set_pwm_channel_duty_cycle(self.motor_pin, 0.0)  # para motores
        # ADAPTAÇÃO: Parar motores e centralizar direção
        print("Parando sistema de controle...")
        
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
        
        navigator.set_led(self.led_r, True)
        navigator.set_led(self.led_g, False)
        navigator.set_led(self.led_b, False)  # Liga o LED azul para indicar que a simulação está ativa
        
        print("Sistema parado")
    
    def emergency_stop(self):
        """Parada de emergência - para imediatamente todos os motores"""
        self.running = False
        self.control_thread = None
        navigator.set_pwm_channel_duty_cycle(self.motor_pin, 0.0)  # para motores
        navigator.set_pwm_channel_duty_cycle(self.led_pin, 1.0)  # Acende LED de emergência (se disponível)
        # ADAPTAÇÃO: Comando de parada de emergência para seus atuadores
        print("PARADA DE EMERGÊNCIA ATIVADA")
        navigator.set_led(self.led_r, not navigator.get_led(self.led_r))
        navigator.set_led(self.led_g, False)
        navigator.set_led(self.led_b, False)
        
        # Exemplo:
        # self.motor_controller.emergency_stop()
        # self.servo_steering.center()