import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='spina_arm_controll',
            executable='serial_controller_node',
            name='serial_controller_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', # Please check USB port
            }]
        ),
        
    ])