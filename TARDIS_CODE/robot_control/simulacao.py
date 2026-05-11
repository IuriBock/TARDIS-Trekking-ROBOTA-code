#!/usr/bin/env python3
"""
Script para simular o robô com o mesmo comportamento visual do robô real.
Gera uma única imagem que é atualizada e sobrescrita.
"""

import time
import math
import random
import listen_data
from send_data import enviar_dados
from control_system import AckermannControlSystem
from data_structures import TrajectoryPoint
import bluerobotics_navigator as navigator
from bluerobotics_navigator import UserLed

def criar_trajetoria_basica():
    """Trajetória básica para teste (reta + curvas 90°)."""
    return [
        (0.0, 0.0, 0.3),
        (2.0, 0.0,  0.4),
        (3.0, 1.0, 0.3),
        (2.0, 2.0,  0.4),
        (0.0, 2.0,  0.3),
        (0.0, 0.0,  0.2),
    ]

def criar_trajetoria_curvas():
    """Trajetória com curvas suaves."""
    traj = [(0.0, 0.0, 0.3)]
    
    # Primeiro trecho reto
    traj.append((2.0, 0.0,  0.3))
    
    # Curva suave (arco de círculo)
    for i in range(1, 10):
        angle = math.radians(i * 10)
        x = 2.0 + 1.0 * math.sin(angle)
        y = 1.0 - 1.0 * math.cos(angle)
        traj.append((x, y, 0.3))
    
    # Reta final
    traj.append((3.0, 1.0,  0.3))
    traj.append((3.0, 2.0, 0.3))
    
    return traj

def criar_trajetoria_quadrado():
    """Trajetória em formato de quadrado com lados de 2m."""
    traj = [
        (0.0, 0.0,  1),      # Ponto inicial
        (2.0, 0.0, 1),       # Lado 1
        (2.0, 2.0,  1),      # Lado 2 (90°)
        (0.0, 2.0, 1),      # Lado 3 (180°)
        (0.0, 0.0, 1),     # Lado 4 (270°)
    ]
    return traj

def criar_trajetoria_retangulo():
    """Trajetória em formato de retângulo (3m x 1.5m)."""
    traj = [
        (0.0, 0.0,  0.4),
        (3.0, 0.0,  0.4),
        (3.0, 1.5,  0.4),
        (0.0, 1.5,  0.4),
        (0.0, 0.0,  0.4),
    ]
    return traj

def criar_trajetoria_espiral():
    """Trajetória em espiral (raio aumenta gradualmente)."""
    traj = [(0.0, 0.0, 0.3)]
    
    raio_inicial = 0.5
    raio_final = 2.5
    num_pontos = 12
    
    for i in range(1, num_pontos + 1):
        raio = raio_inicial + (raio_final - raio_inicial) * (i / num_pontos)
        angulo = i * math.pi / 3  # Incremento de 60°
        
        x = raio * math.cos(angulo)
        y = raio * math.sin(angulo)
        
        traj.append((x, y,  0.3))
    
    return traj

def criar_trajetoria_serpentina():
    """Trajetória em forma de serpentina (zigue-zague)."""
    traj = [(0.0, 0.0, 0.35)]
    
    amplitude = 1.5
    comprimento = 4.0
    num_voltas = 3
    
    for i in range(1, num_voltas * 4 + 1):
        x = i * 0.75
        if x > comprimento:
            x = comprimento
            
        # Movimento senoidal
        y = amplitude * math.sin(i * math.pi / 2)
        
            
        traj.append((x, y, 0.35))
    
    return traj

def criar_trajetoria_aleatoria():
    """Trajetória aleatória com pontos espaçados por pelo menos 0.5m."""
    traj = [(0.0, 0.0,  1.0)]
    
    # Número aleatório de pontos (entre 5 e 20)
    num_pontos = random.randint(5, 20)
    
    x, y = 0.0, 0.0
    angulo_atual = 0.0
    
    for _ in range(num_pontos):
        # Gera distância entre 0.5 e 2.0m
        distancia = random.uniform(0.5, 2.0)
        
        # Gera mudança de ângulo entre -45° e 45° (em radianos)
        # Limitado a 25° por passo para respeitar a limitação do robô
        delta_angulo = random.uniform(-math.pi/3, math.pi/3)  # ±25°
        angulo_atual += delta_angulo
        
        # Calcula novo ponto
        novo_x = x + distancia * math.cos(angulo_atual)
        novo_y = y + distancia * math.sin(angulo_atual)
        
        # Garante que não vai muito longe (limite de 15m)
        if math.sqrt(novo_x**2 + novo_y**2) > 15.0:
            break
            
        traj.append((novo_x, novo_y, random.uniform(0.3, 0.45)))
        x, y = novo_x, novo_y
    
    return traj

def criar_trajetoria_estrela():
    """Trajetória em formato de estrela de 5 pontas."""
    traj = [(0.0, 0.0, 0.35)]
    
    raio = 1.8
    num_pontas = 5
    
    for i in range(1, num_pontas * 2 + 1):
        angulo = i * math.pi / num_pontas
        
        # Alterna entre raio maior e menor para formar estrela
        if i % 2 == 0:
            r = raio * 0.4
        else:
            r = raio
            
        x = r * math.cos(angulo)
        y = r * math.sin(angulo)
        
        traj.append((x, y, 0.35))
    
    # Retorna ao ponto inicial
    traj.append((0.0, 0.0, 0.35))
    
    return traj
def personalizada():
    trajetoria = [  #Trajetória volta a robota
        (0.0, 0.0, 0.0),
        (2.8, 0.0, 1.0),
        (3.3, -2.6, 1.0),
        (1.1, -3.1, 1.0),
        (0.0, -5.6, 1.0),
        (1.9, -6.3, 1.0),
        (3.8, 1.0, 1.0),
        (0.0, 1.6, 1.0),
    ]
    return trajetoria

def validar_trajetoria(traj):
    """Valida se os pontos da trajetória têm espaçamento adequado."""
    if len(traj) < 2:
        return True
    
    pontos_muito_proximos = []
    for i in range(1, len(traj)):
        x1, y1, _ = traj[i-1]
        x2, y2, _ = traj[i]
        distancia = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if distancia < 0.3:
            pontos_muito_proximos.append((i-1, i, distancia))
    
    if pontos_muito_proximos:
        print("\n⚠️  AVISO: Pontos muito próximos detectados:")
        for p1, p2, dist in pontos_muito_proximos[:3]:
            print(f"   Pontos {p1} e {p2}: distância = {dist:.2f}m")
        if len(pontos_muito_proximos) > 3:
            print(f"   ... e mais {len(pontos_muito_proximos)-3} pares")
        print()
    
    return True

def main():
    print("=== SIMULAÇÃO DO ROBÔ ===")
    print("Comportamento igual ao robô real: imagem atualizada em 'simulacao_atual.png'\n")
    
    # Dicionário de trajetórias disponíveis
    trajetorias = {
        '1': ("Básica (reta + curvas 90°)", criar_trajetoria_basica),
        '2': ("Curvas suaves", criar_trajetoria_curvas),
        '3': ("Quadrado (2m x 2m)", criar_trajetoria_quadrado),
        '4': ("Retângulo (3m x 1.5m)", criar_trajetoria_retangulo),
        '5': ("Espiral", criar_trajetoria_espiral),
        '6': ("Serpentina (zigue-zague)", criar_trajetoria_serpentina),
        '7': ("Aleatória", criar_trajetoria_aleatoria),
        '8': ("Estrela de 5 pontas", criar_trajetoria_estrela),
        '9': ("Trajetória personalizada", personalizada),
    }
    
    # Mostra opções
    print("Trajetórias disponíveis:")
    for key, (nome, _) in trajetorias.items():
        print(f"{key}. {nome}")
    
    escolha = input("\nEscolha uma trajetória (1-9): ").strip()
    
    # Seleciona a trajetória
    if escolha in trajetorias:
        _, funcao_traj = trajetorias[escolha]
        trajetoria = funcao_traj()
    else:
        print("Opção inválida! Usando trajetória básica.")
        trajetoria = criar_trajetoria_basica()
    
    # Valida a trajetória
    validar_trajetoria(trajetoria)
    
    # Converte para lista de TrajectoryPoint
    pontos_traj = [TrajectoryPoint(x, y, speed) for x, y, speed in trajetoria]
    
    enviar_dados(f"[Referência]: {trajetoria}")  # Envia a trajetória para o sistema de controle (se necessário)

    
    # Cria sistema em modo simulação
    control = AckermannControlSystem(simulation_mode=True)
    control.load_trajectory(trajetoria)
    
    print(f"\nTrajetória carregada com {len(trajetoria)} pontos")
    print("Simulação em execução... Pressione Ctrl+C para parar\n")
    control.start()
    try:
        print("Iniciando automaticamente")
        while True:
            if listen_data.operacao_ativa():
                if not control.running:
                    print("Iniciando o sistema de controle")
                    control.start() # Inicia o sistema de controle
                    time.sleep(1)
            else:
                if listen_data.emergencia_ativa():
                    control.emergency_stop()   # pode ser chamado repetidamente, sem problema
                    #finalizar_hardware()
                time.sleep(0.1)
            navigator.set_led(UserLed.Led2, True)  # Liga o LED azul para indicar que a simulação está ativa
            navigator.set_led(UserLed.Led1, True)

    except KeyboardInterrupt:
        print("\nSimulação interrompida pelo usuário")
    finally:
        control.stop()
        navigator.set_led(UserLed.Led2, False)  # Liga o LED azul para indicar que a simulação está ativa
        navigator.set_led(UserLed.Led1, False)

if __name__ == "__main__":
    main()