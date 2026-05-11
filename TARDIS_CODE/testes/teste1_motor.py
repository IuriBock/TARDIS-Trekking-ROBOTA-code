import bluerobotics_navigator as navigator
import time

navigator.init() # Inicia navigator
navigator.set_pwm_freq_hz(50)
navigator.set_pwm_enable(True)
try:
    while True:
        navigator.set_pwm_channel_duty_cycle(1,0.18)
        #navigator.set_pwm_channel_duty_cycle(2,1)
        print("em loop")
except KeyboardInterrupt:
    navigator.set_pwm_channel_duty_cycle(2,0.0)
    print("Parada de emergência!")