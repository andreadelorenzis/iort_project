import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'vacuum_bot'

    # -----------------------------
    # RSP: robot state publisher
    # -----------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # -----------------------------
    # Joystick
    # -----------------------------
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg), 'launch', 'joystick.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # -----------------------------
    # Twist mux
    # -----------------------------
    twist_mux_params = os.path.join(get_package_share_directory(pkg), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # -----------------------------
    # Spawn entity in Gazebo
    # -----------------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ],
        output='screen'
    )
        # start choords map house
        #     '-x', '-8.5',
            # '-y', '-1.0',
            # '-z', '0.1',
            # '-R', '0.0',
            # '-P', '0.0',
            # '-Y', '0.0'

    # -----------------------------
    # ROS2 Control node
    # -----------------------------
    controllers = os.path.join(get_package_share_directory(pkg), 'config', 'my_controllers.yaml')
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers],
        output="both",
    )


    # -----------------------------
    # Spawner dei controller
    # -----------------------------
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # -----------------------------
    # Step controller
    # -----------------------------
    step_controller_node = Node(
        package=pkg,
        executable='step_controller_node',
        parameters=[{'simulation': True}, {'use_sim_time': True}],
        output='screen'
    )

    # -----------------------------
    # Launch twist_mux dopo diff_drive
    # -----------------------------
    delayed_twist_mux = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[twist_mux]
        )
    )

    delayed_step_controller_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[step_controller_node]
        )
    )

    # -----------------------------
    # Gazebo bridge nodes
    # -----------------------------
    bridge_params = os.path.join(get_package_share_directory(pkg), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    # -----------------------------
    # Nav2
    # -----------------------------
    nav2_params_file = os.path.join(get_package_share_directory(pkg), 'config', 'nav2_params.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true', 'params_file': nav2_params_file}.items()
    )

    # -----------------------------
    # SLAM toolbox
    # -----------------------------
    slam_params_file = os.path.join(get_package_share_directory(pkg), 'config', 'mapper_params_online_async.yaml')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={'slam_params_file': slam_params_file, 'use_sim_time': 'true'}.items()
    )

    # -----------------------------
    # RViz
    # -----------------------------
    rviz_config_file = os.path.join(get_package_share_directory(pkg), 'config', 'map.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )
    delayed_rviz = TimerAction(period=5.0, actions=[rviz_node])

    # -----------------------------
    # Sequenza: ros2_control_node → spawner → twist_mux → step_controller
    # -----------------------------
    return LaunchDescription([
        control_node,
        rsp,
        joystick,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        delayed_twist_mux,
        # delayed_step_controller_node,
        ros_gz_bridge,
        ros_gz_image_bridge,
        nav2,
        slam_toolbox,
        delayed_rviz
    ])
