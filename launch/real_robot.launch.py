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
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    coverage_config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'opennav_params.yaml'
    )
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )


    # Start robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )])
    )

    # Start multiplexer
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel')]
        )

    # Start step controller
    step_controller_node = Node(
        package=package_name,
        executable='step_controller_node',
        parameters=[{'simulation': True}, {'use_sim_time': False}],
        output='screen'
    )

    ldlidar_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 
                         'launch', 
                         'ldlidar_slam.launch.py'))
        )

    # start nav2/OpenNAV
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 
                         'launch',
                         'opennav_bringup_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'params_file': coverage_config_file}.items())


    # Start the coverage node
    coverage_node = Node(
        package=package_name,
        executable='coverage_nav2_node',
        emulate_tty=True,
        output='screen'
    )
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        step_controller_node,
        ldlidar_slam,
        bringup_cmd,
        coverage_node
    ])
