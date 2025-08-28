import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart


def generate_launch_description():
    package_name='vacuum_bot'
    signals_dir = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'signals'
    )

    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'lifecycle_mgr_slam.yaml'
    )

    # Start robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )])
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            # YAML files
            lc_mgr_config_path  # Parameters
        ]
    )

    # Include LDLidar launch    
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory(package_name),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={
            'node_name': 'ldlidar_node'
        }.items()
    )

    # Fake odom publisher
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # Start step controller
    step_controller_node = Node(
        package=package_name,
        executable='real_step_controller_node',
        parameters=[{'signals_dir': signals_dir}],
        output='screen'
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        lc_mgr_node,
        fake_odom,
        ldlidar_launch,
        step_controller_node
    ])
