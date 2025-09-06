import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node, LifecycleNode

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
    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'final.rviz',
    )
    # SLAM Toolbox configuration for LDLidar
    slam_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'slam_toolbox.yaml'
    )

    # Start multiplexer
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel')]
        )

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path],
        remappings=[('/scan', '/ldlidar_node/scan')]
    )

    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox']
        }]
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
    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'namespace': '', 
            'rviz_config': rviz_config,
            'use_sim_time': 'false'
        }.items())
    
    web_bridge_node = Node(
        package=package_name,
        executable='web_bridge_node',
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        lc_mgr_node,
        slam_toolbox_node,
        twist_mux,
        bringup_cmd,
        coverage_node,
        # web_bridge_node,
        rviz_cmd
    ])
