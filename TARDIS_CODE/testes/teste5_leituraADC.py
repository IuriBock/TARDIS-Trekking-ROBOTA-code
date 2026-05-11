import bluerobotics_navigator as navigator
import time
# Inicializa o módulo
navigator.init()
time.sleep(0.5)

# Lê o valor do canal ADC0 (Ch0)
while True:
    valor_adc0 = navigator.read_adc(navigator.AdcChannel.Ch0)
    print(f"Valor do ADC0: {valor_adc0}")

    # Opcional: Lê todos os canais de uma vez
    dados_adc = navigator.read_adc_all()
    print(f"Valor de todos os canais: {dados_adc.channel}")