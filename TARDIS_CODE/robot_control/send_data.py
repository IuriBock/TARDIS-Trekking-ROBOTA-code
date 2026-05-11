import socket

# --- Configuração do Socket (adicione no início do seu código) ---
def enviar_dados(dado):
    """Função para enviar dados via UDP"""
    try:
        # Cria um socket UDP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Define o endereço e a porta do seu computador 
        SERVER_IP = '192.168.42.192'  # <<<--- MUDE PARA O IP DO SEU COMPUTADOR
        SERVER_PORT = 5005

        sock.sendto(dado.encode(), (SERVER_IP, SERVER_PORT))
        sock.close()
        
    except Exception as e:
        # Se não conseguir enviar, apenas imprime o erro no terminal
        print(f"Erro ao enviar dados: {e}")

# How to use: enviar_dados(dado_gerado)


