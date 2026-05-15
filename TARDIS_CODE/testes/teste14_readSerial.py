import serial
import time
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel
import numpy as np
import matplotlib
# Força o matplotlib a gerar imagens sem precisar de uma interface gráfica ativa (X11)
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
# --- CONFIGURAÇÃO ---
PORTA_SERIAL = '/dev/ttyAMA2' 
BAUD_LIDAR = 115200

# Configurações do Motor (Navigator)
PINO_MOTOR = PwmChannel.Ch4          # Canal PWM da Navigator onde o motor está conectado
TARGET_RPM = 300        # Rotação ideal de operação do LIDAR
K_P = 0.01            # Ganho Proporcional do controle (ajuste se oscilar)
K_I = K_P/1000        # Ganho Integral do controle (ajuste para eliminar erro estacionário)
PWM_MIN = 0.0           # Força mínima para vencer a inércia do motor
PWM_MAX = 0.5           # Limite de segurança (80% do ciclo de trabalho)

# Filtros de Distância (em mm)
DIST_MIN = 0
DIST_MAX = 5000

def salvar_mapa_sonar(leituras):
    """Gera e salva o gráfico circular estilo sonar de 360 graus"""
    # Converte os ângulos de graus para radianos (necessário para o gráfico polar)
    angulos_rad = np.radians(np.arange(360))
    distancias = np.array(leituras)

    # Configuração estética do Sonar (Preto e Verde)
    plt.style.use('dark_background')
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(6, 6))
    
    # Plota os pontos do LIDAR
    # 'go' significa Green Circles (bolinhas verdes). lw=0 remove linhas conectando os pontos
    ax.plot(angulos_rad, distancias, 'go',  markersize=2, alpha=0.7)
    
    # Customização do radar
    ax.set_theta_zero_location('N')  # Define o 0° (Frente) para o Norte/Topo
    ax.set_theta_direction(-1)       # Sentido horário
    ax.set_ylim(0, DIST_MAX)         # Limite máximo de alcance no gráfico
    
    # Cores das grades e textos
    ax.tick_params(colors='#00FF00')
    ax.grid(True, color='#004400', linestyle='--')
    
    # Título estilizado
    plt.title("VARREDURA SONAR 360°", color='#00FF00', pad=20, fontsize=12)
    
    # Salva a imagem no diretório atual
    plt.savefig('mapa_sonar.png', facecolor='black', edgecolor='none', dpi=100)
    plt.close(fig) # Fecha a figura para liberar memória do Raspberry Pi

def main():
    # Array que armazenará a rotação completa (360 graus)
    leituras = [0] * 360
    pwm_atual = 0.5 # Começa aplicando 40% de potência no motor
    erro_integral = 0    # Acumulador para o termo integral do controle
    timestamp_mapa = time.time() # Cronômetro para controle de atualização do mapa
    try:
        # Inicializa o hardware da Navigator
        navigator.init()
        navigator.set_pwm_freq_hz(50)
        navigator.set_pwm_enable(True)
        time.sleep(0.5)
        
        # Abre a porta serial do sensor
        ser = serial.Serial(PORTA_SERIAL, BAUD_LIDAR, timeout=0.1)
        print("Conectado à Navigator e ao LIDAR. Estabilizando rotação...")
        
        timestamp_print = time.time()
        
        navigator.set_pwm_channel_duty_cycle(PINO_MOTOR, pwm_atual)

        while True:
            # Encontra o cabeçalho do pacote (0xFA)
            if ser.read() == b'\xfa':
                index_byte = ser.read()
                if not index_byte:
                    continue
                
                index = ord(index_byte)
                
                # Verifica se é um índice de pacote válido (0xA0 a 0xF9)
                if 0xA0 <= index <= 0xF9:
                    # O pacote restante tem 20 bytes: 
                    # [2 de Velocidade] + [16 de Dados (4 leituras)] + [2 de Checksum]
                    payload = ser.read(20)
                    if len(payload) < 20:
                        continue
                    
                    # 1. CONTROLE DE VELOCIDADE (Malha Fechada)
                    # Extrai a velocidade dos dois primeiros bytes do payload
                    speed_raw = (payload[1] << 8) | payload[0]
                    rpm_atual = speed_raw / 64.0
                    
                    # Cálculo do erro e ajuste do PWM (Algoritmo P)
                    erro = TARGET_RPM - rpm_atual
                    erro_integral += erro  # Acumula o erro para o termo integral
                    up = erro * K_P
                    ui = erro_integral * K_I
                    pwm_atual = up + ui
                    pwm_atual = max(PWM_MIN, min(PWM_MAX, pwm_atual)) # Restringe aos limites
                    
                    # Aplica a nova velocidade no motor através da Navigator
                    navigator.set_pwm_channel_duty_cycle(PINO_MOTOR, pwm_atual)
                    
                    # 2. PROCESSAMENTO DAS DISTÂNCIAS
                    base_angle = (index - 0xA0) * 4
                    
                    for i in range(4):
                        angulo_atual = (360 - (base_angle + i)) % 360
                        # pos pula os 2 bytes iniciais de velocidade. Cada leitura ocupa 4 bytes.
                        pos = 2 + (i * 4) 
                        
                        dist_low = payload[pos]
                        dist_high = payload[pos+1]
                        
                        # O bit mais significativo do byte alto indica erro de leitura do laser
                        erro_leitura = dist_high & 0x80
                        
                        # Reconstrói a distância mascarando os bits de status (usa 0x3F)
                        dist_raw = ((dist_high & 0x3F) << 8) | dist_low
                        
                        # Filtro de validação básica do hardware e range
                        if not erro_leitura and DIST_MIN <= dist_raw <= DIST_MAX:
                            leituras[angulo_atual] = dist_raw
                        else:
                            leituras[angulo_atual] = 0 # Define como 0 se a leitura falhou

                    # Cronômetro para renderizar o mapa (1 em 1 segundo)
                    if time.time() - timestamp_mapa > 0.3:
                        salvar_mapa_sonar(leituras)
                        timestamp_mapa = time.time()

                    # 3. TELEMETRIA (Exibe dados controlados para não sobrecarregar o terminal)
                    if time.time() - timestamp_print > 0.5:
                        status = "ESTÁVEL" if abs(erro) < 8 else "AJUSTANDO"
                        print(f"[LIDAR] RPM: {rpm_atual:.1f} | PWM: {pwm_atual:.2%} | Status: {status} | Dist frente (0°): {leituras[1]}mm")
                        '''n = 0
                        for i in leituras:
                            n+=1
                            if i != 0:
                                print(i,n)
                        '''
                        timestamp_print = time.time()
                        
            else:
                navigator.set_pwm_channel_duty_cycle(PINO_MOTOR, 0.2)

    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário. Desligando...")
    except Exception as e:
        print(f"\nErro durante a execução: {e}")
    finally:
        # Garante o desligamento do motor ao sair do script
        navigator.set_pwm_channel_duty_cycle(PINO_MOTOR, 0.0)
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print("Porta serial fechada e motor desligado com sucesso.")

if __name__ == "__main__":
    main()