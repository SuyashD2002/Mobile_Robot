import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # PACKAGE PATHS
    # ============================================================

    robot_system_dir = os.path.join(
        os.path.expanduser('~'),
        'ros2_robot_ws',
        'src',
        'robot_system'
    )

    gazebo_ros_dir = '/opt/ros/humble/share/gazebo_ros'

    # ============================================================
    # WORLD
    # ============================================================

    world_file = os.path.join(
        robot_system_dir,
        'worlds',
        'slam_environment.world'
    )

    # ============================================================
    # ROBOT URDF
    # ============================================================

    robot_urdf = os.path.join(
        robot_system_dir,
        'urdf',
        'mobile_robot.urdf'
    )

    with open(robot_urdf, 'r') as infp:
        robot_description = infp.read()

    # ============================================================
    # GAZEBO
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description
            }
        ],
        output='screen'
    )

    # ============================================================
    # SPAWN ROBOT
    # ============================================================

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'warehouse_robot',
            '-file',
            robot_urdf,
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.15'
        ],
        output='screen'
    )

    # ============================================================
    # LIDAR NODE
    # ============================================================

    gazebo_lidar_node = Node(
        package='robot_system',
        executable='gazebo_lidar_node',
        name='gazebo_lidar_node',
        output='screen'
    )

    # ============================================================
    # BATTERY NODE
    # ============================================================

    battery_node = Node(
        package='robot_system',
        executable='battery_node',
        name='battery_node',
        output='screen'
    )

    # ============================================================
    # ROBOT CONTROLLER
    # ============================================================

    controller_node = Node(
        package='robot_system',
        executable='robot_controller_node',
        name='robot_controller_node',
        output='screen'
    )

    # ============================================================
    # CMD VEL WATCHDOG
    # ============================================================

    watchdog_node = Node(
        package='robot_system',
        executable='cmd_vel_watchdog',
        name='cmd_vel_watchdog',
        output='screen'
    )

    # ============================================================
    # DASHBOARD
    # ============================================================

    dashboard_node = Node(
        package='robot_system',
        executable='dashboard',
        name='robot_dashboard',
        output='screen'
    )

    # ============================================================
    # EKF
    # ============================================================

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(
                robot_system_dir,
                'config',
                'ekf.yaml'
            )
        ],
        output='screen'
    )

    # ============================================================
    # START SYSTEM NODES AFTER GAZEBO + ROBOT
    # ============================================================

    delayed_system = TimerAction(
        period=6.0,
        actions=[
            gazebo_lidar_node,
            battery_node,
            controller_node,
            watchdog_node,
            dashboard_node,
            ekf_node
        ]
    )

    # ============================================================
    # SPAWN ROBOT AFTER GAZEBO STARTS
    # ============================================================

    delayed_robot = TimerAction(
        period=4.0,
        actions=[
            spawn_robot
        ]
    )

    # ============================================================
    # LAUNCH EVERYTHING
    # ============================================================

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_robot,
        delayed_system
    ])
