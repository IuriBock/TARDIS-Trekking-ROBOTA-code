import bluerobotics_navigator as navigator
from bluerobotics_navigator import UserLed

import time
import sys

try:
    navigator.init()
    print("Navigator inicializado com sucesso.")
except Exception as e:
    print(f"Falha na inicialização: {e}")
    sys.exit(1)

navigator.set_led(UserLed.Led1,True)
navigator.set_led(UserLed.Led2,True)