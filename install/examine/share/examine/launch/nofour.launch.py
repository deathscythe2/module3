from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='examine',
            executable='server',
        
        ),
        Node(
            package='examine',
            executable='client',
        
        ),
        Node(
            package='examine',
            executable='publisher',
        
        ),
        Node(
            package='examine',
            executable='subscriber',
        
        ),
    ]
    )