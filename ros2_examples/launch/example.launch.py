from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_examples',
            executable='minimal_publisher',
            name='talker',
            output='screen'
        ),
        Node(
            package='ros2_examples',
            executable='minimal_subscriber',
            name='listener',
            output='screen'
        )
    ])
