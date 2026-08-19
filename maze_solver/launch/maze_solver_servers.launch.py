from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('linear_speed', default_value='0.18'),
        DeclareLaunchArgument('angular_speed', default_value='0.7'),
        Node(
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
        ),
    ])
