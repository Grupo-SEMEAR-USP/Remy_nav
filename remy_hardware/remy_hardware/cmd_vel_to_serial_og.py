#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial

# Parâmetros do robô
WHEEL_SEPARATION = 0.448   # m
WHEEL_RADIUS = 0.074       # m
MAX_PWM = 255
MAX_LINEAR_SPEED = 0.5     # m/s -> calibrar depois

PPR = 90.0  # pulsos por rotação -> PROVISÓRIO, recalibrar depois
DIST_PER_PULSE = (2 * math.pi * WHEEL_RADIUS) / PPR

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0)  # timeout=0 -> não bloqueante

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Estado da odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_pulsos1 = None
        self.last_pulsos2 = None
        self.last_time = self.get_clock().now()

        self.serial_buffer = ""

        # Timer pra ler a serial periodicamente (20Hz, igual ao envio do ESP)
        self.timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info(f'Conectado ao ESP em {port}')

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * WHEEL_SEPARATION / 2.0)
        v_right = v + (w * WHEEL_SEPARATION / 2.0)

        pwm_left = int(max(-MAX_PWM, min(MAX_PWM, (v_left / MAX_LINEAR_SPEED) * MAX_PWM)))
        pwm_right = int(max(-MAX_PWM, min(MAX_PWM, (v_right / MAX_LINEAR_SPEED) * MAX_PWM)))

        cmd = f"{pwm_left},{pwm_right}\n"
        self.ser.write(cmd.encode())

    def read_serial(self):
        # Lê tudo que estiver disponível no buffer serial
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except Exception:
            return
        if not data:
            return

        self.serial_buffer += data.decode(errors='ignore')

        while '\n' in self.serial_buffer:
            line, self.serial_buffer = self.serial_buffer.split('\n', 1)
            line = line.strip()
            if line.startswith("ODOM,"):
                self.process_odometry(line)

    def process_odometry(self, line):
        parts = line.split(',')
        if len(parts) != 3:
            return
        try:
            pulsos1 = int(parts[1])  # esquerda
            pulsos2 = int(parts[2])  # direita

           # self.get_logger().info(f"Roda Esq (1): {pulsos1} | Roda Dir (2): {pulsos2}")
        except ValueError:
            return

        now = self.get_clock().now()

        if self.last_pulsos1 is None:
            # Primeira leitura, só inicializa
            self.last_pulsos1 = pulsos1
            self.last_pulsos2 = pulsos2
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        delta_p1 = pulsos1 - self.last_pulsos1
        delta_p2 = pulsos2 - self.last_pulsos2
        self.last_pulsos1 = pulsos1
        self.last_pulsos2 = pulsos2
        self.last_time = now

        # Distância percorrida por cada roda
        d_left = delta_p1 * DIST_PER_PULSE
        d_right = delta_p2 * DIST_PER_PULSE

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / WHEEL_SEPARATION

        # Atualiza pose (integração simples)
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        v = d_center / dt
        w = d_theta / dt

        self.publish_odometry(now, v, w)

    def publish_odometry(self, stamp, v, w):
        q = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.write(b"0,0\n")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()