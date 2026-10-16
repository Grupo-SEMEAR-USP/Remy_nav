from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav_dir = FindPackageShare("remy_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = LaunchConfiguration("base_frame")
    nav2_params_file = PathJoinSubstitution([nav_dir, "config", "nav2_params.yaml"])

    # EKF Local
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "publish_tf": True},
            PathJoinSubstitution([nav_dir, "config", "ekf_filter_node.yaml"]),
        ],
        remappings=[],
    )

    # Comentários para eu não me perder XD
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': base_frame,
            'map_frame_id': 'map', #frame pai (aponta)
            'odom_frame_id': 'odom', #frame filho (é apontado)

            'subscribe_rgb': True, #recebe rgb se true
            'subscribe_depth': True, #recebe profundidade se true
            'subscribe_rgbd': False, #recebe as duas de cima já fundidas
            'subscribe_scan_cloud': True, #recebe lidar
            'approx_sync': True, #aproxima o timing do lidar e camera

            'publish_tf': True, #publica map -> odom
            'Reg/Strategy': '2',  # 0 é só visal, 1 só lidar, 2 os dois
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('scan', '/scan_filtered')
        ],
        arguments=['-d'], # -d deleta o banco de dados anterior a cada nova rodada
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
        }.items()
    )

    return LaunchDescription([
        # ---- args geral ------
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),

        ekf_node,
        #rtabmap_node,
        #nav2_launch,
        
    ])