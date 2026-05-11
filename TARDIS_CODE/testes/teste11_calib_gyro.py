import math
import bluerobotics_navigator as navigator
import time
from send_data_teste import enviar_dados
from bluerobotics_navigator import PwmChannel

navigator.init()
time.sleep(0.5)
navigator.set_pwm_freq_hz(50)
navigator.set_pwm_enable(True)

def calibrate_gyro_bias(samples):
    sum_gyro = 0.0
    for s in samples:
        sum_gyro += s
    return sum_gyro / len(samples)

def dinamic_bias(gyro_z, gyro_bias_z=0):
    threshold = 0.02
    bias_alpha = 0.01 # taxa de aprendizado
    if abs(gyro_z) < threshold:
            gyro_bias_z = ( (1 - bias_alpha) * gyro_bias_z + bias_alpha * gyro_z)
            return gyro_bias_z
    return gyro_bias_z

                           
if __name__ == "__main__":
    # Teste simples do estimador de yaw
    t_ant = time.time()
    t0 = time.time()
    yaw = 0.0 

    scale = 360/(355) # fator de ajuste
    bias = 0.011
    drift_rate = 0.017
    gyro_ant = 0.0
    medidas = []
    navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch2, 1)

    try:
        while True:
            navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch1, 0.08)
            gyro_z = navigator.read_gyro().z  # rad/s
            t_atual = time.time()
            dt = t_atual - t_ant
            t_ant = t_atual
            # 1. ESTIMATIVA DE BIAS (quando parado)
            bias = dinamic_bias(gyro_z, bias)
            # 2. Aplica filtragem
            corrected_gyro = (((gyro_z - bias) * scale) + gyro_ant)/2
            gyro_ant = corrected_gyro

            # 3. INTEGRAÇÃO
            yaw -= corrected_gyro * dt
            # 4. NORMALIZAÇÃO [0, 2π)
            yaw = yaw % (2 * math.pi)

            print(f"Yaw:{math.degrees(yaw):.4f}º | {yaw:.4f} rad | Bias:{bias:.5f} | GyroZ:{gyro_z:.5f} | dt={dt:.6f}")
            time.sleep(0.01)
            if yaw > 2*3.14:  
                navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch1, 0.104)
                navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch2, 0.0)
    except KeyboardInterrupt:
        print("Teste finalizado.")
        navigator.set_pwm_channel_duty_cycle(PwmChannel.Ch2, 0.0)
    '''
    for i in range(1000):
        gyro_z = navigator.read_gyro().z  # rad/s
        print(gyro_z)
        medidas.append(gyro_z)
    print("Média para BIAS:")
    bias = calibrate_gyro_bias(medidas)
    print(bias)
    '''
'''
0.011398329585790633
0.010924952785950154
0.010952383354306221
0.010943581595178694
0.01098246375741437
0.011018745326562785
'''

'''
tempo total: 100.00 s
Yaw:-70.3391º | -1.2276 rad

tempo total: 30.00 s
Yaw:-21.3123º | -0.3720 rad
Yaw:-21.0918º | -0.3681 rad 

tempo total: 10.00 s
Yaw:-7.0356º | -0.1228 rad 
Yaw:-7.0138º | -0.1224 rad 
Yaw:-7.1847º | -0.1254 rad
Yaw:-7.0208º | -0.1225 rad 

tempo total: 5.00 s
Yaw:-3.5095º | -0.0613 rad 
Yaw:-3.5150º | -0.0613 rad
Yaw:-3.5266º | -0.0616 rad
Yaw:-3.5465º | -0.0619 rad

'''