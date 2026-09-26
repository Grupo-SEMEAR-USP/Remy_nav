import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import struct
import os
import select

class JoystickReader(Node):
    def __init__(self):
        super().__init__('joystick_reader')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.device_path = '/dev/input/js0'
        self.max_value = 2
        self.deadzone = 1000

        self.axes = {
            1: 0,  # Eixo vertical esquerdo
            3: 0   # Eixo horizontal direito
        }

        if not os.path.exists(self.device_path):
            self.get_logger().error(f"Joystick device {self.device_path} not found.")
            rclpy.shutdown()
            return
        
        self.js = open(self.device_path, 'rb', buffering=0)
        self.get_logger().info("Joystick reader iniciado.")
        

        self.timer_event = self.create_timer(0.001, self.update_axes)
        self.timer_pub = self.create_timer(0.05, self.publish_cmd_vel)

    def update_axes(self):
        if not hasattr(self, 'js') or self.js.closed:
            return
        
        while select.select([self.js], [], [], 0)[0]:
            evbuf = self.js.read(8)
            if not evbuf:
                continue
            time, value, type_, number = struct.unpack("IhBB", evbuf)

            if (type_ & ~0x80) == 0x02:  # JS_EVENT_AXIS
                if number in self.axes:
                    if abs(value) < self.deadzone:
                        self.axes[number] = 0
                    else:
                        scaled_value = int(value * self.max_value / 32767)
                        scaled_value = max(-self.max_value, min(self.max_value, scaled_value))
                        self.axes[number] = scaled_value

    def publish_cmd_vel(self):
        twist = Twist()
        twist.linear.x = float(self.axes[1])
        twist.angular.z = float(self.axes[3])
        self.publisher.publish(twist)

        self.get_logger().info(
            f"Published cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}"
        )

    def destroy_node(self):
        self.get_logger().info("Encerrando leitura do joystick...")
        if hasattr(self, 'js') and not self.js.closed:
            self.js.close()
            self.get_logger().info("Joystick device fechado.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
