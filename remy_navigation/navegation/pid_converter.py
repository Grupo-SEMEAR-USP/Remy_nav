import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import time

class PidController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # --- Parâmetros PID ---
        self.declare_parameter('kp', 0.2)
        self.declare_parameter('ki', 0.11)
        self.declare_parameter('kd', 0.0)
        
        # Limites de Saída (PWM) e Integral
        self.declare_parameter('pwm_min', 30.0)
        self.declare_parameter('pwm_max', 60.0)
        self.declare_parameter('integral_max', 50.0) 

        # Geometria do Robô
        self.declare_parameter('wheel_separation', 0.50) # Distância entre rodas (m)
        self.declare_parameter('wheel_radius', 0.08)     # Raio da roda (m)

        self.declare_parameter('cmd_vel_timeout', 0.5)

        # Estado dos Alvos (Target) e Feedback
        self.target_vel_l = 0.0
        self.target_vel_r = 0.0
        self.current_vel_l = 0.0
        self.current_vel_r = 0.0
        
        # Variáveis para cálculo de velocidade via posição (caso necessário)
        self.last_pos_l = 0.0
        self.last_pos_r = 0.0
        self.last_time_feedback = self.get_clock().now()

        self.last_cmd_vel_time = time.time()

        # Variáveis do PID
        self.prev_err_l = 0.0
        self.prev_err_r = 0.0
        self.integral_l = 0.0
        self.integral_r = 0.0

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Publishers
        # Publica [pwm_esq, pwm_dir] para o serial_controller ler
        self.pub_motor_cmd = self.create_publisher(Float32MultiArray, '/motor_commands', 10)
        
        # Debug: publica o esforço calculado de volta no joint_states para ver no rviz/plot
        self.pub_debug = self.create_publisher(JointState, 'joint_states_pid_debug', 10)

        # Loop de Controle (Ex: 20Hz ou 50Hz)
        self.timer = self.create_timer(0.05, self.control_loop) # 0.05s = 20Hz
        self.last_time_control = time.perf_counter()

    def cmd_vel_callback(self, msg):
        # Converte V linear (m/s) e W angular (rad/s) para velocidade das rodas (rad/s)
        self.last_cmd_vel_time = time.time()

        sep = self.get_parameter('wheel_separation').value
        rad = self.get_parameter('wheel_radius').value

        v = msg.linear.x
        w = msg.angular.z

        # Cinemática diferencial
        self.target_vel_l = (v - (w * sep / 2.0)) / rad
        self.target_vel_r = (v + (w * sep / 2.0)) / rad

        # Debug: Log the target velocities for left and right wheels
        self.get_logger().info(f"Target Velocities -> Left: {self.target_vel_l:.2f}, Right: {self.target_vel_r:.2f}")
        
    def joint_state_callback(self, msg):
        # Tenta extrair as velocidades atuais publicadas pelo serial_controller
        # Espera nomes: "wheel_left_joint", "wheel_right_joint"
        try:
            if "wheel_left_joint" in msg.name and "wheel_right_joint" in msg.name:
                idx_l = msg.name.index("wheel_left_joint")
                idx_r = msg.name.index("wheel_right_joint")

                pos_l = msg.position[idx_l]
                pos_r = msg.position[idx_r]

                # Calcular velocidade real baseada na variação de posição / tempo
                now = self.get_clock().now()
                dt = (now - self.last_time_feedback).nanoseconds / 1e9

                if dt > 0:
                    self.current_vel_l = (pos_l - self.last_pos_l) / dt
                    self.current_vel_r = (pos_r - self.last_pos_r) / dt
                    
                    self.last_pos_l = pos_l
                    self.last_pos_r = pos_r
                    self.last_time_feedback = now

        except ValueError:
            pass

    def calculate_pid(self, target, current, prev_err, integral, dt):
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        max_i = self.get_parameter('integral_max').value

        error = target - current

        # Proporcional
        p_term = kp * error

        # Integral
        integral += ki * error * dt
        # Anti-windup (clampar integral)
        integral = max(min(integral, max_i), -max_i)

        # Derivativo
        d_term = 0.0
        if dt > 0:
            d_term = kd * (error - prev_err) / dt
        
        output = p_term + integral + d_term
        
        return output, error, integral
    
    def check_cmd_vel_timeout(self):
        # Verifica se passou do tempo limite
        timeout = self.get_parameter('cmd_vel_timeout').value
        if (time.time() - self.last_cmd_vel_time) > timeout:
            # Se timeout, ZERA os alvos e reseta integrais para evitar solavancos
            self.target_vel_l = 0.0
            self.target_vel_r = 0.0
            self.integral_l = 0.0
            self.integral_r = 0.0

    def control_loop(self):
        now = time.perf_counter()
        dt = now - self.last_time_control
        self.last_time_control = now

        # Evita divisão por zero ou saltos grandes
        if dt <= 0 or dt > 0.5:
            return
        
        self.check_cmd_vel_timeout()

        if abs(self.target_vel_l) < 0.001 and abs(self.target_vel_r) < 0.001:
            self.integral_l = 0.0
            self.integral_r = 0.0
            self.prev_err_l = 0.0
            self.prev_err_r = 0.0

        # PID Esquerda
        out_l, self.prev_err_l, self.integral_l = self.calculate_pid(
            self.target_vel_l, self.current_vel_l, self.prev_err_l, self.integral_l, dt
        )

        # PID Direita
        out_r, self.prev_err_r, self.integral_r = self.calculate_pid(
            self.target_vel_r, self.current_vel_r, self.prev_err_r, self.integral_r, dt
        )

        # Publicar comando para o motor (Float array)
        msg = Float32MultiArray()
        msg.data = [float(out_l), float(out_r)] 
        self.pub_motor_cmd.publish(msg)

        # Debug (Opcional)
        debug_msg = JointState()
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.name = ["wheel_left_joint", "wheel_right_joint"]
        debug_msg.effort = [float(out_l), float(out_r)]
        debug_msg.velocity = [self.current_vel_l, self.current_vel_r]
        self.pub_debug.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()