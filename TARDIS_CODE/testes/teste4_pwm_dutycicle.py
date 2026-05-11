import bluerobotics_navigator as navigator
#from bluerobotics_navigator import PwmChannel

import time

pin = 1

navigator.init()
navigator.set_pwm_freq_hz(50)
navigator.set_pwm_enable(True)
time.sleep(0.5)
'''
for i in range(10):
    navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch1, i/10)
    time.sleep(0.5)
'''
# navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch1, 0.5)

while True:
    duty = float(input("Digite o duty cycle (0 a 10) ou 'sair' para encerrar: "))
    if duty == 'sair':
        break
    navigator.set_pwm_channel_duty_cycle(pin, float(duty/10))
print("finalizado")

# Servo Tardis varia entre aproximadamente 0.12 e 0.08 de duty cycle (12% a 8%) para os extremos, e 0.105 - 0.106 para o centro. 