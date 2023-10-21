# ~/ros2_ws/src/fleet_management/launch/fleet_management.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the fleet management server
        Node(
            package='fleet_management',
            executable='fleet_management_server',
            name='fleet_management_server',
            output='screen',
        ),

        # Launch the fleet management client
        Node(
            package='fleet_management',
            executable='fleet_management_client',
            name='fleet_management_client',
            output='screen',
        ),
    ])

