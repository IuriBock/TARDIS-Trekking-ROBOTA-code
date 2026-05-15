import serial

# Configurações
PORTA_SERIAL = '/dev/ttyAMA2' 
BAUD_LIDAR = 115200
DIST_MIN = 150
DIST_MAX = 2500 # Aumentado para teste

def main():
    try:
        ser = serial.Serial(PORTA_SERIAL, BAUD_LIDAR, timeout=1)
        leituras = [0] * 360
        
        print("Iniciando leitura bruta do LIDAR...")

        while True:
            # Sincroniza com o cabeçalho 0xFA
            if ser.read() == b'\xfa':
                index_byte = ser.read()
                if not index_byte: continue
                
                index = ord(index_byte)
                
                # O índice do pacote vai de 0xA0 até 0xF9
                if 0xA0 <= index <= 0xF9:
                    # Lê o restante do pacote (velocidade + 4 leituras + checksum)
                    # Total do pacote são 22 bytes. Já lemos 2 (FA e Index). Faltam 20.
                    payload = ser.read(20)
                    if len(payload) < 20: continue

                    base_angle = (index - 0xA0) * 4
                    
                    for i in range(4):
                        angulo_atual = (base_angle + i) % 360
                        pos = i * 4 + 2 # Pula os bytes de velocidade (2 bytes iniciais)
                        
                        # Extração da distância
                        dist_low = payload[pos]
                        dist_high = payload[pos+1]
                        
                        # Bit de erro e distância
                        erro = dist_high & 0x80
                        dist_raw = ((dist_high & 0x3F) << 8) | dist_low # Máscara correta 0x3F

                        if not erro and DIST_MIN < dist_raw < DIST_MAX:
                            leituras[angulo_atual] = dist_raw
                            if angulo_atual == 0: # Teste para imprimir apenas um ângulo específico
                                print(f"{angulo_atual} | {dist_raw}")
                            # Imprime todos os ângulos válidos para verificar a cobertura
                            #print(f"{angulo_atual} | {dist_raw}")
                        else:
                            leituras[angulo_atual] = 0

    except Exception as e:
        print(f"Erro: {e}")
    finally:
        if 'ser' in locals(): ser.close()

if __name__ == "__main__":
    main()