# hardware_interface.py
"""
Arquivo exclusivo para comunicação com o hardware real do robô.
"""
import time
import threading
import RPi.GPIO as GPIO
from data_structures import SensorData

import bluerobotics_navigator as navigator
from bluerobotics_navigator import UserLed


# ============================================
# CONFIGURAÇÃO DO LED
# ============================================
led_b = UserLed.Led2
led_r = UserLed.Led3
led_g = UserLed.Led1

# ============================================v
# CONFIGURAÇÃO DO ENCODER COM INTERRUPÇÃO
# ============================================
PIN_ENCODER = 18   # GPIO 18 (PWM0 no conector AUX)

# Variáveis globais para o encoder (thread-safe)
_encoder_posicao = 0
_encoder_lock = threading.Lock()

def _encoder_callback(channel):
    """Callback chamado nas bordas do pino encoder. Atualiza a posição."""
    global _encoder_posicao
    # Lê a direção (pino B) - ajuste conforme o comportamento do seu encoder
    _encoder_posicao += 1  # Incrementa a posição (ajuste para decremento se necessário)

def inicializar_encoder():
    """Configura os pinos e ativa a interrupção."""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(PIN_ENCODER, GPIO.BOTH, callback=_encoder_callback)
    print(f"Encoder inicializado")

# ============================================
# INICIALIZAÇÃO DO NAVIGATOR E ENCODER
# ============================================
def inicializar_hardware():
    navigator.init() # Inicia navigator
    navigator.set_led(led_r, True)
    print("Inicializando Navigator...")
    time.sleep(0.5)  # Espera o sensor estabilizar
    navigator.set_pwm_freq_hz(50)
    navigator.set_pwm_enable(True)
    inicializar_encoder()  # <-- ativa a contagem por interrupção
    time.sleep(0.5)  # Espera o sensor estabilizar
    navigator.set_led(led_r, False)
    navigator.set_led(led_b, True)  


# ============================================
# FUNÇÕES DE BAIXO NÍVEL (acesso direto ao hardware)
# ============================================

def ler_acelerometro():
    """
    Lê os dados do acelerômetro embutido na NAVIGATOR. (from the ICM20689 chip’s accelerometer).
    """
    accel = navigator.read_accel()
    ax = accel.x
    ay = accel.y
    az = accel.z 
    return (ax, ay, az)  # (ax, ay, az) em m/s²

def ler_giroscopio():
    """
    Lê os dados do giroscópio embutido na NAVIGATOR. (from the ICM20689 chip’s gyroscope).
    """
    gyro = navigator.read_gyro()
    gx = gyro.x
    gy = gyro.y
    gz = gyro.z
    return (gx, gy, gz)  # (gx, gy, gz) em rad/s

def ler_magnetometro():
    """
    Lê os dados do magnetômetro embutido na NAVIGATOR. (from the onboard Ak09915 magnetometer).
    """
    mag = navigator.read_mag()
    mx = mag.x 
    my = mag.y
    mz = mag.z
    return (mx, my, mz)  # (mx, my, mz) em µT

def ler_encoder():
    """
    Lê a quantidade de pulsos do encoder
    """
    with _encoder_lock:
        return _encoder_posicao

# ============================================
# FUNÇÃO PRINCIPAL QUE INTEGRA TUDO
# ============================================

def ler_todos_sensores():
    """
    Chama todas as funções específicas e monta o objeto SensorData
    Esta função é chamada pelo programa principal
    """
    # Cria objeto vazio
    dados = SensorData()
    
    # Lê acelerômetro
    ax, ay, az = ler_acelerometro()
    dados.accel_x = ax
    dados.accel_y = ay
    dados.accel_z = az
    
    # Lê giroscópio
    gx, gy, gz = ler_giroscopio()
    dados.gyro_x = gx
    dados.gyro_y = gy
    dados.gyro_z = gz
    
    # Lê magnetômetro
    mx, my, mz = ler_magnetometro()
    dados.mag_x = mx
    dados.mag_y = my
    dados.mag_z = mz
    
    # Lê encoder
    dados.encoder_rear = ler_encoder()
    
    # Timestamp
    dados.timestamp = time.time()
    
    return dados

def finalizar_hardware():
    """Limpeza dos GPIOs (chamar ao final do programa)."""
    navigator.set_led(led_r, False)
    navigator.set_led(led_g, False)
    navigator.set_led(led_b, False)
    GPIO.cleanup()