"""Start action servers, then (optionally) the maze client.

The Gazebo maze launch must already be running in another terminal:

    ros2 launch maze_control maze_simulation_tb3.launch.py
    ros2 launch maze_solver maze_solve.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_client = LaunchConfiguration('run_client')

    servers = Node(
        package='maze_solver',
        executable='action_servers.py',
        name='maze_action_servers',
        output='screen',
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
        }],
    )

    client = Node(
        package='maze_solver',
        executable='maze_client.py',
        name='maze_client',
        output='screen',
        parameters=[{
            'wall_open_sec': 6.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('linear_speed', default_value='0.18'),
        DeclareLaunchArgument('angular_speed', default_value='0.7'),
        DeclareLaunchArgument(
            'run_client',
            default_value='true',
            description='If true, start solve_maze after the action servers.',
        ),
        servers,
        TimerAction(
            period=3.0,
            actions=[client],
            condition=IfCondition(run_client),
        ),
    ])
