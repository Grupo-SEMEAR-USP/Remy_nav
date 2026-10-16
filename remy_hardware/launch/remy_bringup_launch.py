from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    description_dir = get_package_share_directory('remy_description')
    urdf_file = os.path.join(description_dir, 'urdf', 'remy.urdf.xacro')
    rviz_config = os.path.join(
        get_package_share_directory('remy_hardware'), 'config', 'remy_slam.rviz'
    )

    lidar_usb = LaunchConfiguration('lidar_usb')
    esp_usb = LaunchConfiguration('esp_usb')

    # Publica as tfs das juntas do robo
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Nem vou explicar de tão obvio
    declare_esp_usb = DeclareLaunchArgument(
        'esp_usb',
        default_value='/dev/ttyUSB1',
        description='Porta serial do ESP'
    )

    # Agente do microros pra comunicar com a esp32
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', esp_usb, '-b', '115200']
    )

    # Roda o codigo wheel_odometry.py
    odometry_node = Node(
        package='remy_hardware',
        executable='wheel_odometry', 
        name='odometry_node'
    )

    # Nem vou explicar de tão obvio²
    declare_lidar_usb = DeclareLaunchArgument(
        'lidar_usb',
        default_value='/dev/ttyUSB0',
        description='Porta serial do lidar'
    )

    # Launch lidar a2m8
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a2m8_launch.py')
        ]),
        launch_arguments={
            'serial_port': lidar_usb,
            'serial_baudrate': '115200',
            #'frame_id': 'laser'
            #'scan_mode': 'Standard'
        }.items()
    )

    # Filtro do lidar, remove os pontos da parte traseira
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

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'enable_gyro': 'true', 'enable_accel': 'true', 'align_depth.enable': 'true'}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        declare_esp_usb,
        declare_lidar_usb,
        robot_state_publisher,
        joint_state_publisher,
        sllidar_launch,
        laser_filter,
        microros_agent,
        odometry_node,
        realsense_launch,
        rviz,
        
    ])