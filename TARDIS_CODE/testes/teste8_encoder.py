import RPi.GPIO as GPIO
import time
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel

PIN_ENC = 18
PULSOS_POR_REVOLUCAO = 8  # Ajuste conforme o seu encoder
pulsos = 0
start_time = time.time()
ultimo_tempo = 0.0
rpm = 0.0
delta_t = 0.0
pulsos_ant = 0

# Configurações iniciais
ultima_leitura_tempo = time.time()

navigator.init() # Inicia navigator
navigator.set_pwm_freq_hz(50)
navigator.set_pwm_enable(True)
print("inicializando navigator...")
time.sleep(1)

def callback(channel):
    global ultimo_tempo, rpm, delta_t, pulsos
    
    tempo_atual = time.time() - start_time

    delta_t = tempo_atual - ultimo_tempo
    
    if delta_t > 0.001:  # Ignora ruído
        # Calcula RPM instantâneo
        pulsos += 1
        ultimo_tempo = tempo_atual   

    
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_ENC, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(PIN_ENC, GPIO.BOTH, callback=callback)

try:
    navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch2, 1.0)
    while True:
        tempo_atual = time.time()
        if pulsos == pulsos_ant:
            if (tempo_atual - ultima_leitura_tempo) > 1.0:
                delta_t = 0.0
        else:
            # Se houve pulso, o delta_t já deve ter sido calculado 
            # (Idealmente por uma interrupção). 
            # Aqui, apenas atualizamos a referência para a próxima checagem de parada.
            ultima_leitura_tempo = tempo_atual
        
        # 2. Cálculo do RPM Instantâneo
        # Fórmula: 60 segundos divididos pelo tempo que levaria uma volta completa
        if delta_t > 0:
            # O tempo de uma volta completa é: delta_t * pulsos_por_revolucao
            rpm_inst = 60.0 / (delta_t * PULSOS_POR_REVOLUCAO)
        else:
            rpm_inst = 0.0
        
        # 3. Cálculo de Voltas Totais
        voltas = pulsos / PULSOS_POR_REVOLUCAO

        # Exibição
        print(f"Pulsos: {pulsos} | "
            f"Velocidade: {rpm_inst:.2f} RPM | RPS: {rpm_inst/60:.2f} | "
            f"Delta T: {delta_t:.4f} s | "
            f"Voltas: {voltas:.2f}")

        pulsos_ant = pulsos

        time.sleep(1)  # Seu código aqui

except KeyboardInterrupt:
    navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch2, 0.0)

    GPIO.cleanup()