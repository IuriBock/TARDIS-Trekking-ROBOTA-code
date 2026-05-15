import serial
import threading
import time
import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel

class LidarDriver:
    def __init__(self, porta='/dev/ttyAMA2', pino_motor=PwmChannel.Ch4):
        self.porta = porta
        self.pino_motor = pino_motor
        self.target_rpm = 300
        
        self.distancias = [0] * 360
        self.lock = threading.Lock()
        
        self.rodando = False
        self.thread = None

    def _controle_motor(self, payload, erro_integral, kp=0.005, ki=0.00005):
        """Calcula o PWM para manter 300 RPM"""
        speed_raw = (payload[1] << 8) | payload[0]
        rpm_atual = speed_raw / 64.0
        
        erro = self.target_rpm - rpm_atual
        erro_integral += erro
        up = erro * kp
        ui = erro_integral * ki
        U = up + ui
        pwm = max(0.12, min(0.2, U))
        
        print(f"RPM: {rpm_atual:.1f}, PWM: {pwm:.3f}")
        navigator.set_pwm_channel_duty_cycle(self.pino_motor, pwm)
        return erro_integral

    def _processar_leituras(self):
        """Loop de leitura ultra-otimizado com descarte de buffer acumulado"""
        erro_integral = 0
        last_packet_time = time.time()
        
        try:
            # Diminuímos o timeout para a leitura ser instantânea
            ser = serial.Serial(self.porta, 115200, timeout=0.02)
            
            # Limpa qualquer lixo inicial do buffer
            ser.reset_input_buffer()
            
            while self.rodando:
                # SOLUÇÃO DO DELAY: Se houver mais de 2 pacotes acumulados esperando na fila (22 bytes * 2),
                # nós simplesmente dropamos tudo e pegamos o dado fresco do "agora".
                if ser.in_waiting > 44:
                    ser.reset_input_buffer()
                
                byte = ser.read()
                if byte == b'\xfa':
                    index_byte = ser.read()
                    if not index_byte: continue
                    index = ord(index_byte)
                    
                    if 0xA0 <= index <= 0xF9:
                        payload = ser.read(20)
                        if len(payload) < 20: continue
                        
                        # Atualiza o cronômetro de pacotes válidos recebidos
                        last_packet_time = time.time()
                        
                        # Ajusta o motor do LIDAR
                        erro_integral = self._controle_motor(payload, erro_integral)
                        
                        # Extrai as distâncias
                        base_angle = (index - 0xA0) * 4
                        with self.lock:
                            for i in range(4):
                                angulo = (360 - (base_angle + i)) % 360
                                pos = 2 + (i * 4)
                                dist_raw = ((payload[pos+1] & 0x3F) << 8) | payload[pos]
                                self.distancias[angulo] = dist_raw if not (payload[pos+1] & 0x80) else 0
                                
                # SOLUÇÃO DO TRAVAMENTO DO BARRAMENTO:
                # Só tenta reajustar o motor em modo de segurança se o LIDAR sumir por mais de 0.5s.
                # Isso impede que bytes desalinhados fiquem spammando comandos lentos de I2C.
                elif time.time() - last_packet_time > 0.5:
                    navigator.set_pwm_channel_duty_cycle(self.pino_motor, 0.13)
                    last_packet_time = time.time() # Reseta o timer para não floodar
                    
        except Exception as e:
            print(f"[LIDAR ERROR] {e}")
        finally:
            navigator.set_pwm_channel_duty_cycle(self.pino_motor, 0.0)
            if 'ser' in locals() and ser.is_open:
                ser.close()

    def iniciar(self):
        self.rodando = True
        self.thread = threading.Thread(target=self._processar_leituras, daemon=True)
        self.thread.start()
        print("[SISTEMA] Driver do LIDAR em execução (Modo Real-Time ativo).")

    def parar(self):
        self.rodando = False
        if self.thread: 
            self.thread.join(timeout=1.0)

    def pegar_distancias(self):
        with self.lock:
            print(self.distancias)
            return list(self.distancias)