# main.py
import time
import threading
from control_system import AckermannControlSystem
from hardware_interface import inicializar_hardware,ler_todos_sensores,finalizar_hardware 

import listen_data 
from send_data import enviar_dados
from trajetoria_gravada import trajetoria

def main():
    # Inicializa o hardware
    inicializar_hardware()

    # Cria o sistema de controle
    control = AckermannControlSystem()

    # Carrega a trajetória (X ,Y, velocidade)
    trajetoria_man1 = [  #Trajetória volta a robota
        (0.0, 0.0, 0.0),
        (2.3, 0.0, 0.7),
        (2.9, -1.0, 0.5),
        (2.4, -1.8, 0.5),
        (1.1, -2.0, 0.5),
        (0.0, -5.6, 0.5),
        (1.9, -6.3, 0.5),
        (3.8, 1.0, 0.5),
        (0.0, 1.6, 0.5),
    ]
    trajetoria_man2 = [  #Trajetória volta a robota
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 2.0),
    ]
    #trajetoria = trajetoria_man1
    control.load_trajectory(trajetoria)
    enviar_dados(f"[Referência]: {trajetoria}")  # Envia a trajetória para o sistema de controle (se necessário)

    # ============================================
    # THREAD DE LEITURA DOS SENSORES REAIS
    # ============================================
    def loop_sensores():
        """
        Esta thread executa em paralelo com o controle
        Lê os sensores a cada 10ms e envia para o sistema
        """
        while control.running:
            # Lê TODOS os sensores de uma vez
            dados_sensores = ler_todos_sensores()  

            # Envia para o sistema de controle
            control.update_sensors(dados_sensores)  # ← método do control_system
            
            # Aguarda 10ms (100 Hz)
            time.sleep(0.01)
    
    # Inicia a thread de sensores
    thread_sensores = threading.Thread(target=loop_sensores)
    thread_sensores.daemon = True

    # Inicia o listener de comandos (em background)
    listen_data.iniciar_listener()

    # ============================================
    # CONTROLE DO ROBÔ
    # ============================================
    try:
        print("Iniciando automaticamente")
        while True:
            if listen_data.operacao_ativa():
                if not control.running:
                    print("Iniciando o sistema de controle")
                    control.start() # Inicia o sistema de controle
                    thread_sensores = threading.Thread(target=loop_sensores)
                    thread_sensores.daemon = True
                    thread_sensores.start()  # Inicia a thread de sensores
                    time.sleep(1)
            else:
                if listen_data.emergencia_ativa():
                    control.emergency_stop()   # pode ser chamado repetidamente, sem problema
                    thread_sensores = None  # A thread de sensores irá parar sozinha no próximo loop
                time.sleep(0.2)
        
    except KeyboardInterrupt:
        print("\nParada de emergência!")
        control.emergency_stop()
        finalizar_hardware()
    print("Programa encerrado")

if __name__ == "__main__":
    main()