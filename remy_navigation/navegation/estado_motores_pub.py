#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")
        self.joint_pub = self.create_publisher(JointState,"joint_states", 10) 

        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("porta_esp", '/dev/pts/4')

        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.porta_esp = self.get_parameter("porta_esp").get_parameter_value().string_value

        self.get_logger().info("Using wheel radius %d" % self.baud_rate)
        self.get_logger().info("Using wheel separation %s" % self.porta_esp)
        
        self.ser = serial.Serial(self.porta_esp, self.baud_rate)
        self.timer = self.create_timer(0.01, self.ler_serial)#100vezes/seg
            
    def ler_serial(self):
        if self.ser.in_waiting > 0:  # Check if there is data in the buffer
            line = self.ser.readline().decode('utf-8').rstrip()  # Read and decode the data
            self.get_logger().info("Received: "+ line)
            self.publicar(line)


# Open the virtual serial port for reading
 # Replace with the correct virtual port

    # rm 24.9 54.8 [24.9 rotacoes] 1 rot = 2pi rad
    def publicar(self, linha_lida:str):
        array = linha_lida.split()
        if(len(array)==2):
            if(array[0]=="rm"):
                self.pos_mot_e = float(array[1])
                self.pos_mot_d = float(array[2])
                self.pos_rad_mot_e = self.pos_mot_e * 2 * math.pi
                self.pos_rad_mot_d = self.pos_mot_d * 2 * math.pi
                msg = JointState()
                time = self.get_clock().now().to_msg()
                msg.header.stamp=time
                msg.position[1] = self.pos_rad_mot_e # a ordem ta certa
                msg.position[0] = self.pos_rad_mot_d # a ordem ta certa
                self.joint_pub.publish(msg)

def main():
    rclpy.init()

    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
