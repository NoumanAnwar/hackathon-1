import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get URDF path
    # In a real Docusaurus setup, this would be relative to the built site,
    # but for local testing with RViz2, we point to the static directory.
    urdf_path = os.path.join(
        os.getcwd(), # This will be the project root when launched from there
        'book_frontend',
        'static',
        'urdf',
        'simple_arm.urdf'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_path]
    )

    # RViz2 node
    rviz_config_dir = os.path.join(
        get_package_share_directory('urdf_tutorial'),
        'rviz'
    )
    rviz_config_file = LaunchConfiguration('rviz_config', default=os.path.join(rviz_config_dir, 'urdf.rviz'))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file,
            description='Full path to the RVIZ config file to use'
        ),
        robot_state_publisher_node,
        rviz_node
    ])
