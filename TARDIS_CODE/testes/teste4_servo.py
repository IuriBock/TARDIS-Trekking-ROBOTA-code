import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel
import time

navigator.init() # Inicia navigator
navigator.set_pwm_freq_hz(50)
navigator.set_pwm_enable(True)

navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch1, 0.08)
try:
    time.sleep(0.5)  # Aguarda um pouco para garantir que o sistema esteja pronto
    navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch1, 0.08)

except KeyboardInterrupt:
    navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch2,0.0)
    print("Parada de emergência!")