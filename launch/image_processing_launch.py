from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_processor',
            executable='image_publisher',
            name='image_publisher',
            output='screen'
        ),
        Node(
            package='image_processor',
            executable='image_subscriber',
            name='image_subscriber',
            output='screen'
        ),
    ])
