from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                '/home/sarah/Documentos/HOME/remy_navegation/src/navegation/config/ekf_config.yaml',
                {'use_sim_time': True}
            ],

        )
    ])