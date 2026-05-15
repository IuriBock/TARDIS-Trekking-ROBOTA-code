import serial
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel
import time

# --- CONFIGURAÇÃO ---
PORTA_SERIAL = '/dev/ttyAMA2'
PINO_MOTOR = PwmChannel.Ch4          # Pino PWM da Navigator onde o motor está ligado
TARGET_RPM = 280        # Rotação ideal
PWM_MIN = 0.0           # 0%
PWM_MAX = 0.5           # 50% (limite de segurança)
K_P = 0.01            # Ganho do controle (ajuste se oscilar muito)
K_I = K_P/100        # Ganho integral (ajuste para eliminar erro estacionário)

navigator.init() # Inicia navigator
navigator.set_pwm_freq_hz(50)
navigator.set_pwm_enable(True)
print("Inicializando navigator...")
time.sleep(0.5)


def main():
    pwm_atual = 0.5 # Começa com 50% de força
    erro_integral = 0    # Acumulador para o termo integral

    try:
        ser = serial.Serial(PORTA_SERIAL, 115200, timeout=1)
        print(f"Estabilizando motor em {TARGET_RPM} RPM...")
        navigator.set_pwm_channel_duty_cycle(PINO_MOTOR, pwm_atual)

        while True:
            if ser.read() == b'\xfa':
                index_byte = ser.read()
                if not index_byte: continue
                
                # Lê os próximos 2 bytes que contêm a velocidade (Speed)
                speed_data = ser.read(2)
                if len(speed_data) < 2: continue
                
                # Calcula RPM atual
                rpm_atual = ((speed_data[1] << 8) | speed_data[0]) / 64
                
                # --- Lógica de Controle (P) ---
                erro = TARGET_RPM - rpm_atual
                erro_integral += erro  # Acumula o erro para o termo integral
                up = erro * K_P
                ui = erro_integral * K_I
                pwm_atual = up + ui
                
                # Limita o PWM para não queimar o motor ou travar
                pwm_atual = max(PWM_MIN, min(PWM_MAX, pwm_atual))
                
                navigator.set_pwm_channel_duty_cycle(PINO_MOTOR, pwm_atual)

                # Lê o restante do pacote (20 bytes) para limpar o buffer
                ser.read(18) 

                if abs(erro) < 5:
                    status = "ESTÁVEL"
                else:
                    status = "AJUSTANDO"

                print(f"RPM: {rpm_atual:.1f} | PWM: {pwm_atual:.2%}| Status: {status}")

    except Exception as e:
        print(f"Erro: {e}")
    finally:
        navigator.set_pwm_channel_duty_cycle(PINO_MOTOR, 0) # Para o motor ao sair
        if 'ser' in locals(): ser.close()

if __name__ == "__main__":
    main()