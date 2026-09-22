from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    robot_system_dir = get_package_share_directory('robot_system')
    gazebo_dir = get_package_share_directory('gazebo_ros')

    robot_urdf = os.path.join(
        robot_system_dir,
        'urdf',
        'mobile_robot.urdf'
    )

    obstacle_sdf = os.path.join(
        robot_system_dir,
        'worlds',
        'obstacle.sdf'
    )

    # Read URDF for robot_state_publisher
    with open(robot_urdf, 'r') as infp:
        robot_description = infp.read()

    # =========================================================
    # GAZEBO
    # =========================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_dir,
                'launch',
                'gazebo.launch.py'
            )
        )
        spawn_obstacle = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
                   '-file', obstacle_sdf,
                   '-entity', 'obstacle'
    ],
    ...
)
    )

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

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

    # =========================================================
    # ROBOT
    # =========================================================

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-file', robot_urdf,
            '-entity', 'warehouse_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15'
        ],
        output='screen'
    )

    # =========================================================
    # OBSTACLE
    # =========================================================

    spawn_obstacle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_obstacle',
        arguments=[
            '-file', obstacle_sdf,
            '-entity', 'obstacle'
        ],
        output='screen'
    )

    # =========================================================
    # GAZEBO LIDAR
    # =========================================================

    gazebo_lidar_node = Node(
        package='robot_system',
        executable='gazebo_lidar_node',
        name='gazebo_lidar_node',
        output='screen'
    )

    # =========================================================
    # BATTERY
    # =========================================================

    battery_node = Node(
        package='robot_system',
        executable='battery_node',
        name='battery_node',
        output='screen'
    )

    # =========================================================
    # CONTROLLER
    # =========================================================

    controller_node = Node(
        package='robot_system',
        executable='robot_controller_node',
        name='robot_controller_node',
        output='screen'
    )

    # =========================================================
    # WATCHDOG
    # =========================================================

    watchdog_node = Node(
        package='robot_system',
        executable='cmd_vel_watchdog',
        name='cmd_vel_watchdog',
        output='screen'
    )

    # =========================================================
    # DASHBOARD
    # =========================================================

    dashboard_node = Node(
        package='robot_system',
        executable='dashboard',
        name='robot_dashboard',
        output='screen'
    )

    # =========================================================
    # EKF
    # =========================================================

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

    # =========================================================
    # START ROBOT AFTER 5 SECONDS
    # =========================================================

    delayed_robot = TimerAction(
        period=5.0,
        actions=[
            spawn_robot
        ]
    )

    # =========================================================
    # START OBSTACLE AFTER 8 SECONDS
    # =========================================================

    delayed_obstacle = TimerAction(
        period=8.0,
        actions=[
            spawn_obstacle
        ]
    )

    # =========================================================
    # START ROS SYSTEM AFTER 11 SECONDS
    # =========================================================

    delayed_system = TimerAction(
        period=11.0,
        actions=[
            gazebo_lidar_node,
            battery_node,
            controller_node,
            watchdog_node,
            dashboard_node,
            ekf_node
        ]
    )

    # =========================================================
    # MASTER LAUNCH
    # =========================================================

    return LaunchDescription([
        gazebo,

        robot_state_publisher,

        delayed_robot,

        delayed_obstacle,

        delayed_system
    ])
