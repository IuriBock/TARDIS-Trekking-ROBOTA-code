#!/usr/bin/env python3
"""
Gravador de trajetória interativo – versão simplificada.
Configure as opções abaixo e execute o script.
Pressione Ctrl+C para parar e salvar a trajetória.
"""

import time
import signal
import threading
import os
import sys

from control_system import AckermannControlSystem
import hardware_interface as hw
from send_data import enviar_dados
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel
from hardware_interface import inicializar_hardware, finalizar_hardware

# ============================================================
# CONFIGURAÇÕES – EDITAR CONFORME NECESSÁRIO
# ============================================================
MODO = "manual"          # "auto" ou "manual"
INTERVALO_AUTO = 0.5   # segundos entre pontos (modo automático)
VELOCIDADE_DESEJADA = 1.0  # m/s (padrão para todos os pontos)
NOME_ARQUIVO_SAIDA = "trajetoria_gravada.py"  # arquivo gerado
# ============================================================

# Inicialização do hardware (mesmo do cria_trajetoria.py)
inicializar_hardware()
time.sleep(0.5)  # estabiliza
navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch1, 0.1053)  # Neutro

class SimpleTrajectoryRecorder:
    def __init__(self):
        self.points = []
        self.running = False
        self.control_system = AckermannControlSystem(simulation_mode=False, web_server=False)

    def feed_sensors(self):
        """Alimenta os sensores reais no sistema de controle"""
        while self.running:
            sensor_data = hw.ler_todos_sensores()
            self.control_system.update_sensors(sensor_data)
            time.sleep(0.01)

    def start(self):
        self.running = True
        # Thread de leitura dos sensores
        feeder = threading.Thread(target=self.feed_sensors)
        feeder.daemon = True
        feeder.start()
        # Thread de fusão de sensores (já existe no control_system)
        self.control_system.running = True
        processor = threading.Thread(target=self.control_system.process_sensors)
        processor.daemon = True
        processor.start()
        time.sleep(0.5)  # estabiliza
        print("Sensores iniciados. Robô pronto para gravação.\n")

    def record_point(self):
        """Registra um ponto com o estado atual"""
        state = self.control_system.current_state
        point = (state.x, state.y, VELOCIDADE_DESEJADA)
        self.points.append(point)
        enviar_dados(f"[Posição]: x={state.x:.3f} y={state.y:.3f} yaw={state.yaw:.3f}")
        print(f"[{len(self.points):3d}] x={state.x:6.2f}  y={state.y:6.2f}  "
              f"yaw={state.yaw*57.2958:6.1f}°  vel_desej={VELOCIDADE_DESEJADA:.2f} m/s")

    def run_auto(self):
        """Modo automático – grava a cada INTERVALO_AUTO segundos"""
        print(f"Modo automático: gravando a cada {INTERVALO_AUTO} s. Pressione Ctrl+C para encerrar.\n")
        while self.running:
            self.record_point()
            # Espera o intervalo, mas verificando a cada 0.1s se deve parar
            for _ in range(int(INTERVALO_AUTO * 10)):
                if not self.running:
                    break
                time.sleep(0.1)

    def run_manual(self):
        """Modo manual – grava ao pressionar ENTER"""
        print("Modo manual: pressione ENTER para gravar um ponto. Digite 'q' e ENTER para sair.\n")
        while self.running:
            cmd = input().strip().lower()
            if cmd == 'q':
                break
            else:
                self.record_point()

    def save(self):
        """Salva os pontos num arquivo Python no mesmo diretório do script"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.join(script_dir, NOME_ARQUIVO_SAIDA)
        with open(full_path, 'w', encoding='utf-8') as f:
            f.write("# Trajetória gravada (gravador interativo)\n")
            f.write("# Formato: (x, y, yaw_rad, speed_mps)\n\n")
            f.write("trajetoria = [\n")
            for i, (x, y, speed) in enumerate(self.points):
                f.write(f"    ({x:.6f}, {y:.6f}, {speed:.6f})")
                if i < len(self.points) - 1:
                    f.write(",\n")
                else:
                    f.write("\n")
            f.write("]\n")
        print(f"\nTrajetória salva em: {full_path} ({len(self.points)} pontos)")

    def stop(self):
        self.running = False
        self.control_system.running = False
        self.control_system.stop()
        finalizar_hardware()

    def run(self):
        self.start()
        try:
            if MODO == "auto":
                self.run_auto()
            else:
                self.run_manual()
        except KeyboardInterrupt:
            print("\n\nGravação interrompida pelo usuário.")
        finally:
            self.stop()
            if self.points:
                self.save()
            else:
                print("Nenhum ponto foi gravado.")


if __name__ == "__main__":
    recorder = SimpleTrajectoryRecorder()
    recorder.run()