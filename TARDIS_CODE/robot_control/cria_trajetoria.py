#!/usr/bin/env python3
"""
Gravador de trajetória para robô Ackermann.

Reutiliza o AckermannControlSystem existente.
Funcionamento:
- Uma thread lê os sensores reais (via hardware_interface) e chama update_sensors()
- O process_sensors (do control_system) faz a fusão e atualiza current_state
- A thread principal grava current_state em intervalos fixos
- Ao final, salva a lista de pontos no mesmo diretório do script

Uso:
    python trajectory_recorder.py --interval 1.0 --output minha_trajetoria.py
"""

import time
import signal
import argparse
import threading
import os
from typing import List, Tuple

# Importa os módulos existentes
from control_system import AckermannControlSystem
import hardware_interface as hw
from send_data import enviar_dados
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel

VELOCIDADE = 1
navigator.init() # Inicia navigator
navigator.set_pwm_freq_hz(50)
navigator.set_pwm_enable(True)
navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch1, 0.1053)  # Neutro
class TrajectoryRecorder:
    def __init__(self, interval_sec: float = 1.0):
        self.interval = interval_sec
        self.points: List[Tuple[float, float, float, float]] = []
        self.running = False
        
        # Cria o sistema de controle em modo REAL
        self.control_system = AckermannControlSystem(simulation_mode=False, web_server=False)
        
        # Threads
        self.sensor_feeder_thread = None
        self.sensor_processing_thread = None

    def feed_sensors(self):
        """Lê os sensores reais e alimenta o control_system via update_sensors()."""
        print("[Feeder] Iniciando leitura de sensores...")
        while self.running:
            # Lê todos os sensores do hardware
            sensor_data = hw.ler_todos_sensores()
            # Coloca na fila do sistema de controle
            self.control_system.update_sensors(sensor_data)
            # Aguarda um pouco para não sobrecarregar (100 Hz é suficiente)
            time.sleep(0.01)
        print("[Feeder] Encerrado.")

    def start(self):
        """Inicia as threads de sensores e a gravação."""
        self.running = True
        
        # Thread que alimenta os sensores
        self.sensor_feeder_thread = threading.Thread(target=self.feed_sensors)
        self.sensor_feeder_thread.daemon = True
        self.sensor_feeder_thread.start()
        
        # Thread de processamento dos sensores (fusão) – já existe no control_system
        # Precisamos rodar o process_sensors em segundo plano
        self.control_system.running = True
        self.sensor_processing_thread = threading.Thread(target=self.control_system.process_sensors)
        self.sensor_processing_thread.daemon = True
        self.sensor_processing_thread.start()

        # Aguarda um pouco para a primeira fusão acontecer
        time.sleep(0.5)
        print("Sistema de sensores iniciado. Gravando trajetória...")

    def record_loop(self):
        """Loop principal que grava os pontos em intervalos regulares."""
        last_record_time = time.time()
        next_record_time = last_record_time + self.interval
        
        print(f"Gravando a cada {self.interval} segundos. Pressione Ctrl+C para parar.")
        print("Mova o robô manualmente ou com teleoperação.\n")
        
        while self.running:
            now = time.time()
            if now >= next_record_time:
                state = self.control_system.current_state
                point = (state.x, state.y, state.speed)
                enviar_dados(f"[Posição Atual]: x={state.x:.3f}| y={state.y:.3f}")
                print("[Dados enviados]")
                self.points.append(point)
                
                print(f"[{len(self.points):3d}] x={state.x:6.2f}  y={state.y:6.2f}  "
                      f"speed={state.speed:5.2f} m/s")
                
                next_record_time = now + self.interval
            
            time.sleep(0.01)  # evita uso excessivo de CPU

    def save_to_file(self, filename: str):
        """
        Salva a lista de pontos em um arquivo Python.
        O arquivo será criado no MESMO DIRETÓRIO onde este script está localizado.
        """
        # Obtém o diretório do script atual
        script_dir = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.join(script_dir, filename)
        
        with open(full_path, 'w', encoding='utf-8') as f:
            f.write("# Trajetória gravada automaticamente\n")
            f.write("# Formato: (x, y, speed_mps)\n\n")
            f.write("trajetoria = [\n")
            for i, (x, y, speed) in enumerate(self.points):
                #speed = VELOCIDADE
                f.write(f"    ({x:.6f}, {y:.6f}, {speed:.6f})")
                if i < len(self.points) - 1:
                    f.write(",\n")
                else:
                    f.write("\n")
            f.write("]\n")
        
        print(f"\nTrajetória salva em: {full_path}")
        print(f"Total de pontos: {len(self.points)}")

    def stop(self):
        """Para todas as threads e o sistema."""
        self.running = False
        self.control_system.running = False
        
        if self.sensor_feeder_thread and self.sensor_feeder_thread.is_alive():
            self.sensor_feeder_thread.join(timeout=1.0)
        if self.sensor_processing_thread and self.sensor_processing_thread.is_alive():
            self.sensor_processing_thread.join(timeout=1.0)
        
        # Desliga atuadores (segurança)
        self.control_system.stop()
        print("Gravador encerrado.")

def signal_handler(recorder: TrajectoryRecorder):
    def handler(sig, frame):
        print("\nInterrompendo gravação...")
        recorder.stop()
    return handler
     
def main():
    parser = argparse.ArgumentParser(description="Gravador de trajetória para robô Ackermann")
    parser.add_argument('--interval', '-i', type=float, default=0.5,
                        help='Intervalo entre pontos (segundos) [padrão: 1.0]')
    parser.add_argument('--output', '-o', type=str, default='trajetoria_gravada.py',
                        help='Nome do arquivo de saída (será salvo no mesmo diretório do script)')
    args = parser.parse_args()

    # Inicializa o hardware (já feito pelo import de hardware_interface)
    # Mas precisamos garantir que o encoder e navigator estejam prontos
    print("Inicializando hardware...")
    # O módulo hardware_interface já executa navigator.init() e inicializa o encoder.
    # Se necessário, um pequeno delay para estabilizar:
    time.sleep(0.5)

    recorder = TrajectoryRecorder(interval_sec=args.interval)
    recorder.start()

    # Configura tratamento de sinal para Ctrl+C
    signal.signal(signal.SIGINT, signal_handler(recorder))

    try:
        recorder.record_loop()
    except Exception as e:
        print(f"Erro durante gravação: {e}")
    finally:
        if recorder.points:
            recorder.save_to_file(args.output)
        else:
            print("Nenhum ponto foi gravado.")
        recorder.stop()
        hw.finalizar_hardware()
        print("Fim.")

if __name__ == "__main__":
    main()