#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Point
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# Parâmetros do robô
WHEEL_SEPARATION = 0.448   # m
WHEEL_RADIUS = 0.074       # m
PPR = 90.0  # pulsos por rotação -> PROVISÓRIO, recalibrar depois
DIST_PER_PULSE = (2 * math.pi * WHEEL_RADIUS) / PPR

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Subscreve os pulsos que vêm do micro-ROS (ESP32)
        self.sub = self.create_subscription(Point, '/encoder_ticks', self.ticks_callback, 10)
        
        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Estado da odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_pulsos1 = None
        self.last_pulsos2 = None
        self.last_time = self.get_clock().now()

        self.get_logger().info('Nó de Odometria iniciado. A aguardar /encoder_ticks...')

    def ticks_callback(self, msg: Point):
        # O Point transmite como float (double no C++), então convertemos para inteiro
        pulsos1 = int(msg.x)  # esquerda
        pulsos2 = int(msg.y)  # direita

        now = self.get_clock().now()

        if self.last_pulsos1 is None:
            # Primeira leitura, inicializa as variáveis de estado
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

        # Atualiza a pose no mapa (integração simples)
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        v = d_center / dt
        w = d_theta / dt

        self.publish_odometry(now, v, w)

    def publish_odometry(self, stamp, v, w):
        q = yaw_to_quaternion(self.theta)

        # Publicar no tópico /odom
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

        # Atualizar a árvore de transformadas (TF2) odom -> base_footprint
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
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()