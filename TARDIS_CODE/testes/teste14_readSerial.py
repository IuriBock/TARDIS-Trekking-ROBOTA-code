import serial
import time

# --- CONFIGURAÇÃO ---
# Porta configurada para a SERIAL 4 (ttyAMA2) da Navigator
PORTA_SERIAL = '/dev/ttyAMA2' 
BAUD_LIDAR = 115200

ANGULO_MIN = 0
ANGULO_MAX = 360
DIST_MIN = 150
DIST_MAX = 1500
TOLERANCIA_VIZINHO = 150

# Array para armazenar as 360 leituras
leituras = [0] * 360

def main():
    try:
        # Inicializa a conexão serial com o LIDAR
        ser = serial.Serial(PORTA_SERIAL, BAUD_LIDAR, timeout=0.1)
        
        # Este print inicial só aparece uma vez para você saber que conectou
        print("Conectado ao LIDAR na porta (SERIAL 4 da Navigator)...")
        
        while True:
            # Procura pelo cabeçalho 0xFA
            if ser.read() == b'\xfa':
                index_byte = ser.read()
                if not index_byte:
                    continue
                
                index = ord(index_byte)

                # Verifica se o índice está no range válido (0xA0 a 0xF9)
                if 0xA0 <= index <= 0xF9:
                    # Lê os próximos 20 bytes do pacote
                    p = ser.read(20)
                    if len(p) == 20:
                        base_angle = (index - 0xA0) * 4

                        for i in range(4):
                            angulo_atual = (base_angle + i) % 360
                            pos = i * 4 
                            
                            # Combina os bytes para distância (Little Endian)
                            dist_raw = ((p[pos+1] & 0x1F) << 8) | p[pos]
                            erro = p[pos+1] & 0x80

                            # 1. Filtro de hardware e range
                            if erro or dist_raw < DIST_MIN or dist_raw > DIST_MAX:
                                dist_raw = 0
                            
                            leituras[angulo_atual] = dist_raw

                            # 2. Lógica do Filtro de Vizinhos
                            angulo_anterior = (angulo_atual - 1) % 360
                            dist_final = 0

                            if leituras[angulo_atual] > 0 and leituras[angulo_anterior] > 0:
                                if abs(leituras[angulo_atual] - leituras[angulo_anterior]) < TOLERANCIA_VIZINHO:
                                    dist_final = leituras[angulo_atual]

                            # 3. Saída de Dados (FORMATO PARA O PROCESSING)
                            if ANGULO_MIN <= angulo_atual <= ANGULO_MAX:
                                if angulo_atual == 90:
                                    print(f"{angulo_atual} | {dist_final}")

                                if dist_final > 0:
                                    # Imprime "angulo,distancia"
                                    print(f"{angulo_atual} | {dist_final}")
                                #else:
                                    # Imprime "angulo,0" para limpar o mapa
                                    #print(f"{angulo_atual} | 0")

    except KeyboardInterrupt:
        print("\nEncerrando o sensor...")
    except Exception as e:
        print(f"Erro na execução: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Porta serial fechada.")

if __name__ == "__main__":
    main()