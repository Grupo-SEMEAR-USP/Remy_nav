from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = LaunchConfiguration("base_frame")
    #vale a pena usar ground truth para comparação?

    imu_topic = LaunchConfiguration("imu_topic")

    # EKF Local
    ekf_yaml = LaunchConfiguration("ekf_yaml")
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "publish_tf": True},
            ekf_yaml,
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

            'subscribe_rgb': False, #recebe rgb se true
            'subscribe_depth': False, #recebe profundidade se true
            'subscribe_rgbd': True, #recebe as duas de cima já fundidas
            'subscribe_scan_cloud': True, #recebe lidar
            'approx_sync': True, #aproxima o timing do lidar e camera

            'publish_tf': True, #publica map -> odom
            'Reg/Strategy': '2',  # 0 é só visal, 1 só lidar, 2 os dois
        }],
        remappings=[],
        arguments=['-d'],
    )


    # Grupo de nós de odometria dos sensores
    sensor_odometry_group = GroupAction([        
        # EKF Local
        ekf_node,

        # RTAB Map SLAM
    #    rtabmap_node,
    ])

    return LaunchDescription([
        # ---- args geral ------
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),

        # ---- args EKF ----
        DeclareLaunchArgument("ekf_yaml", default_value=PathJoinSubstitution([FindPackageShare("remy_navigation"), "config", "ekf_filter_node.yaml"])),

        # Sensor odometry com gps
        sensor_odometry_group,
    ])