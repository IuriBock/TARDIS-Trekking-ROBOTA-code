import RPi.GPIO as GPIO
import time

PIN_ENC = 18
PULSOS_POR_REVOLUCAO = 8  # Ajuste conforme o seu encoder
pulsos = 0
start_time = time.time()
ultimo_tempo = 0.0
rpm = 0.0
delta_t = 0.0
pulsos_ant = 0

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
    while True:
        if pulsos == pulsos_ant:
            delta_t = 0.0

        tempo_atual = time.time() - start_time
        
        voltas = pulsos / PULSOS_POR_REVOLUCAO  # Supondo que cada volta completa gera 4 pulsos
        if delta_t == 0:
            rpm = 0.0
        else:
            rpm = 60/ (delta_t * PULSOS_POR_REVOLUCAO)
        rpm_med = (voltas / tempo_atual) * 60.0
        print(f"Pulsos: {pulsos} | Voltas: {voltas:.2f} | Velocidade: {rpm:.2f} RPM | Tempo: {tempo_atual:.2f} s | Tempo entre os pulsos: {delta_t:.4f} s | RPM médio: {rpm_med:.2f} RPM")
        
        pulsos_ant = pulsos
        time.sleep(1)  # Seu código aqui

except KeyboardInterrupt:
    GPIO.cleanup()