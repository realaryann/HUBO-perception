from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='real_tf_broadcaster',
            executable='real_tf_broadcaster',
            name='real_tf_broadcaster',
        ),
        Node(
            package='pointcloud_practice',
            executable='pointcloud_practice',
            name='pointcloud_practice',
            output='screen',
            parameters=[
                {'MIN_CLUSTER_SIZE' : 50},
                {'MAX_CLUSTER_SIZE' : 500},
                {'TABLE_HEIGHT' : 10.0},
                {'TOLERANCE' : 0.01},
                {'REMOVE_FLOOR' : False},
                {'LIFETIME' : 5e9},
            ],
        ),
        Node(
            package='pointcloud_pipe',
            name='pointcloud_pipe',
            executable='pipe'
        ),
        Node(
            package='image_parsing',
            name='image_parsing',
            executable='classifier',
        ),
        # namespace_launch_arg,
        # tf_prefix_launch_arg,
        # container,
    ])