import socket
import subprocess
import subprocess
import re
import ipaddress

def obter_ip_local():
    """Obtém o endereço IP local (LAN) da máquina."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Conecta a um IP público (não precisa enviar dados)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip

# ============================================
# DESCOBERTA AUTOMÁTICA DO IP DO ROBÔ
# ============================================
def obter_ip_robo():
    """Tenta descobrir o IP do Raspberry Pi na rede."""
    return "192.168.42.1"  # fallback

def encontrar_ip_pi_por_mac():
    """Executa o comando 'arp -a' e procura por MACs de Raspberry Pi."""
    try:
        resultado = subprocess.run(["arp", "-a"], capture_output=True, text=True, check=True)
        linhas = resultado.stdout.splitlines()
    except Exception as e:
        print(f"❌ Erro ao executar arp -a: {e}")
        return None

if __name__ == "__main__":
    ip_local = obter_ip_local()
    print(f"Endereço IP local: {ip_local}")