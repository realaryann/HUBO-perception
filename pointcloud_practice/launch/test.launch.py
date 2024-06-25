from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions

def generate_launch_description():
    # namespace = '/camera'

    # container = launch_ros.actions.ComposableNodeContainer(
    #         name='container',
    #         namespace=namespace,
    #         package='rclcpp_components',
    #         executable='component_container',
    #         composable_node_descriptions=[
    #             # Just the driver
    #             launch_ros.descriptions.ComposableNode(
    #                 package='openni2_camera',
    #                 plugin='openni2_wrapper::OpenNI2Driver',
    #                 name='driver',
    #                 parameters=[{'depth_registration': True},
    #                             {'use_device_time': True}],
    #                 namespace=namespace,
    #             ),
    #         ],
    #         output='screen',
    # )

    return LaunchDescription([
        Node(
            package='sim_tf_broadcaster',
            executable='sim_tf_broadcaster',
            name='sim_tf_broadcaster',
        ),
        Node(
            package='pointcloud_practice',
            executable='pointcloud_practice',
            name='pointcloud_practice',
            output='screen',
            parameters=[
                {'MIN_CLUSTER_SIZE' : 1},
                {'MAX_CLUSTER_SIZE' : 10000},
                {'TABLE_HEIGHT' : 2.1},
            ],
        ),
        # container,
    ])