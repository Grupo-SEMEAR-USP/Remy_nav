import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState #, Imu
from std_msgs.msg import Float32MultiArray
import serial
import math
import time

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_sender')

        self.get_logger().info(f'Use Sim Time: {self.get_parameter("use_sim_time").value}')

        # Configuração da Serial
        self.serial_port = '/dev/ttyUSB0' # Verifique se é esta porta
        self.baud_rate = 115200

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10) 
        # self.imu_pub = self.create_publisher(Imu, "imu", 10) 

        # Inicialização da Serial
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Serial connected to {self.serial_port}")
            time.sleep(2) # Aguarda o reboot da ESP (comum em algumas placas)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            rclpy.shutdown()
            return

        # Subscriber para receber comandos do PID
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor_commands',
            self.listener_callback,
            10
        )

        # Variáveis para integração de posição (Odometria das rodas)
        self.pos_l_rad = 0.0
        self.pos_r_rad = 0.0
        self.last_time_update = self.get_clock().now()

        # Timer para ler a serial freqüentemente
        self.timer_pub = self.create_timer(0.01, self.ler_serial) # 100Hz

    def listener_callback(self, msg):
        """
        Recebe [esforço_esq, esforço_dir] do PID.
        O código da ESP espera: "pwm1,dir1,pwm2,dir2"
        """
        if len(msg.data) >= 2:
            pid_l = msg.data[0] # Esquerda (Motor 1 na ESP?)
            pid_r = msg.data[1] # Direita (Motor 2 na ESP?)

            # --- Lógica de Conversão PID -> PWM/Dir ---
            pwm_l = int(min(255, max(0, abs(pid_l))))
            pwm_r = int(min(255, max(0, abs(pid_r))))

            dir_l = 1 if pid_l >= 0 else 0
            dir_r = 1 if pid_r >= 0 else 0

            # Formato ESP: "PWM,pwm1,dir1,pwm2,dir2"
            message = f"PWM,{pwm_l},{dir_l},{pwm_r},{dir_r}\n"
            
            try:
                self.ser.write(message.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")

    def ler_serial(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                # Opcional: Logar tudo para debug inicial
                # self.get_logger().info(f"Raw Serial: {line}")
                
                if line.startswith("RPM"):
                    self.processar_dados_motores(line)
                # elif line.startswith("IMU"):
                #     pass 
        except Exception as e:
            self.get_logger().warn(f"Erro leitura serial: {e}")

    def processar_dados_motores(self, linha: str):
        """
        Processa linha no formato: "RPM,rpm1,rpm2"
        """
        parts = linha.split(',')
        if len(parts) == 3:
            try:
                # Extrai RPM
                rpm_m1 = float(parts[1]) # Motor Esquerdo (Assumindo M1=Esq)
                rpm_m2 = float(parts[2]) # Motor Direito (Assumindo M2=Dir)

                # Converte RPM para Rad/s
                # rad/s = RPM * 2pi / 60  => RPM * 0.10472
                vel_rad_m1 = rpm_m1 * (2 * math.pi / 60.0)
                vel_rad_m2 = rpm_m2 * (2 * math.pi / 60.0)

                # --- Integração para Posição (Necessário para JointState) ---
                now = self.get_clock().now()
                dt = (now - self.last_time_update).nanoseconds / 1e9
                self.last_time_update = now

                # Evita saltos gigantes se o dt for instável na inicialização
                if dt < 1.0: 
                    self.pos_l_rad += vel_rad_m1 * dt
                    self.pos_r_rad += vel_rad_m2 * dt

                # Publica JointState
                msg = JointState()
                msg.header.stamp = now.to_msg()
                msg.name = ["rev2", "rev1"]
                
                msg.position = [self.pos_l_rad, self.pos_r_rad]
                msg.velocity = [vel_rad_m1, vel_rad_m2]
                msg.effort = [] 
                
                self.joint_pub.publish(msg)

            except ValueError:
                self.get_logger().warn("Erro de conversão numérica na serial")

        # --- SEÇÃO IMU COMENTADA ---
        # def publicar_imu(self, parts):
        #     if len(parts) == X: # Ajustar conforme protocolo IMU futuro
        #         msg_imu = Imu()
        #         msg_imu.header.stamp = self.get_clock().now().to_msg()
        #         msg_imu.header.frame_id = "imu_link"
        #         msg_imu.linear_acceleration.x = float(parts[...])
        #         ...
        #         self.imu_pub.publish(msg_imu)

def main(args=None):
    rclpy.init(args=args)
    node = SerialController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()