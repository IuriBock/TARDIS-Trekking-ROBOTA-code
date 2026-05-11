# listen_data.py
import socket
import threading
import time
from capta_ip_robo import obter_ip_local, obter_ip_robo

_CMD_PORT = 5006
_emergency = False      # True = parada de emergência ativa
_active = False          # True = robô pode se mover (se não estiver em emergência)
_lock = threading.Lock()

def _listener():
    """Thread que escuta comandos UDP."""
    global _emergency, _active
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', _CMD_PORT))
    print(f"[listen_data] Escutando comandos na porta {_CMD_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            cmd = data.decode().strip()
            with _lock:
                if cmd == "START":
                    _emergency = False
                    _active = True
                    print("[listen_data] Comando START recebido")
                elif cmd == "EMERGENCY_STOP":
                    _emergency = True
                    _active = False
                    print("[listen_data] Comando EMERGENCY_STOP recebido")
                else:
                    print(f"[listen_data] Comando desconhecido: {cmd}")
        except Exception as e:
            print(f"[listen_data] Erro: {e}")
            time.sleep(0.1)

def iniciar_listener():
    """Inicia a thread de escuta (deve ser chamada uma vez no início)."""
    t = threading.Thread(target=_listener, daemon=True)
    t.start()

def emergencia_ativa():
    """Retorna True se o robô está em emergência."""
    with _lock:
        return _emergency

def operacao_ativa():
    """Retorna True se o robô está autorizado a se mover (não emergência e não pausado)."""
    with _lock:
        return _active and not _emergency

def resetar_emergencia():
    global _emergency, _active
    """Limpa o estado de emergência (útil se quiser reiniciar manualmente)."""
    with _lock:
        _emergency = False
        _active = True

# Opcional: função para enviar confirmação de volta (não usado agora)