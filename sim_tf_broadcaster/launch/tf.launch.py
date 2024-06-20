from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sim',
            parameters=[
                {'X': 0.67},
                {'Y': -0.001},
                {'Z': 1.25},
                {'YAW': -1.57},
            ],
        ),
    ])