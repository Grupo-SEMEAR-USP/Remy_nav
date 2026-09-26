#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__('ps4_teleop_node')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.max_speed = 0.25
        self.max_turn = 0.75

        self.timer = self.create_timer(0.05, self.publish_cmd)
        self.joy_msg = None

    def joy_callback(self, msg):
        self.joy_msg = msg

    def publish_cmd(self):
        if self.joy_msg is None:
            return

        twist = Twist()

        # Analógico esquerdo vertical (eixo 1) → linear
        # Analógico esquerdo horizontal (eixo 0) → angular
        twist.linear.x = self.joy_msg.axes[1] * self.max_speed
        twist.angular.z = self.joy_msg.axes[0] * self.max_turn

        # L2 (botão 6) para turbo
        if self.joy_msg.buttons[5]:  # R1
            twist.linear.x *= 2.0
            twist.angular.z *= 2.0

        # Quadrado (botão 3) para parada imediata
        if self.joy_msg.buttons[3]:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PS4TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
