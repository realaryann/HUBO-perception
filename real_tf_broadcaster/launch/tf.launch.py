from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_tf_broadcaster',
            executable='sim_tf_broadcaster',
            name='sim_tf_broadcaster',
        ),
    ])