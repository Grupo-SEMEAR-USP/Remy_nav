from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    lidar_usb = LaunchConfiguration('lidar_usb')
    esp_usb = LaunchConfiguration('esp_usb')

    declare_lidar_usb = DeclareLaunchArgument(
        'lidar_usb',
        default_value='/dev/ttyUSB0',
        description='Porta serial do lidar'
    )

    declare_esp_usb = DeclareLaunchArgument(
        'esp_usb',
        default_value='/dev/ttyUSB1',
        description='Porta serial do ESP'
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a2m8_launch.py')
        ]),
        launch_arguments={
            'serial_port': lidar_usb,
            'serial_baudrate': '115200',
            #'scan_mode': 'Standard'
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(
                get_package_share_directory('remy_hardware'), 'config', 'slam_params.yaml'
            )
        }.items()
    )

    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', esp_usb, '-b', '115200']
    )

    # antes aqui era o esp bridge
    odometry_node = Node(
        package='remy_hardware',
        executable='cmd_vel', 
        name='odometry_node'
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=['0.15', '0', '0.05', '0.0', '0', '0.0', 'base_footprint', 'laser']
    )

    rviz_config = os.path.join(
    get_package_share_directory('remy_hardware'), 'config', 'remy_slam.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    )

    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        # name='laser_filter',
        parameters=[os.path.join(
            get_package_share_directory('remy_hardware'), 'config', 'laser_filter.yaml'
        )],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ]
    )

    return LaunchDescription([
        declare_lidar_usb,
        declare_esp_usb,
        sllidar_launch,
        microros_agent,
        odometry_node,
        lidar_tf,
        slam_launch,
        rviz,
        laser_filter,
    ])