from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    share_dir = get_package_share_directory('urdf2_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'urdf2.xacro') 

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=xacro_file,
        description="Absolute path to robot URDF file"
    )

    robot_description = ParameterValue(Command(["xacro ", 
                                                LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(share_dir).parent.resolve())
            ]
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),

        launch_arguments=[
            ("gz_args", [" -v 4", " -r", " empty.sdf"])
        ]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "remy"]
    )

    # Spawnar os controllers após o Gazebo subir
    joint_state_broadcaster_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            parameters=[{"use_sim_time": True}]
        )]
    )

    diff_drive_spawner = TimerAction(
        period=15.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller"],
            parameters=[{"use_sim_time": True}]
        )]
    )

    # Ponte para o Clock e outros tópicos
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        bridge,
    ])
