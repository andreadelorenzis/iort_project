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

    # Start robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )])
    )


    # Start step controller
    step_controller_node = Node(
        package=package_name,
        executable='real_step_controller_node',
        parameters=[{'signals_dir': signals_dir}],
        output='screen'
    )

    ldlidar_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 
                         'launch', 
                         'ldlidar_slam.launch.py'))
        )

    # Launch them all!
    return LaunchDescription([
        rsp,
        step_controller_node,
        ldlidar_slam,
    ])
