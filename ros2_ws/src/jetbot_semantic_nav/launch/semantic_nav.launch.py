"""For an existing case whose supervisor does NOT already own these nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    case=LaunchConfiguration('case')
    return LaunchDescription([DeclareLaunchArgument('case'),
        Node(package='jetbot_semantic_nav',executable='semantic_mapper',arguments=['--case',case]),
        Node(package='jetbot_semantic_nav',executable='semantic_goal_manager',arguments=['--case',case])])
