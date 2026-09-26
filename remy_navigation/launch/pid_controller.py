from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Argumentos
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Porta Serial'),
        
        # Nó de Comunicação Serial (Hardware)
        Node(
            package='navegation',
            executable='serial_controller',
            name='serial_interface',
            output='screen',
            parameters=[{'serial_port': LaunchConfiguration('port')}]
        ),

        # Nó PID (Controle)
        Node(
            package='navegation',
            executable='pid_converter', 
            name='pid_converter',
            output='screen',
            parameters=[{
                'kp': 0.15,
                'ki': 0.7,
                'kd': 0.001,
                'wheel_separation': 0.50, 
                'wheel_radius': 0.08      
            }]
        )
    ])