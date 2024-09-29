import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    package_prefix = get_package_share_directory("ros_coneslayer")
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_prefix, 'launch', 'coneslayer_publisher.launch.py'))),

        ComposableNodeContainer(
            name=name + '_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                    ComposableNode(
                        package="ros_coneslayer",
                        plugin="ros_coneslayer::SpatialBB",
                        name="spatial_bb_node",
                        remappings=[
                                    ('nn/spatial_detections', '/color/spatial_detections'),
                                    ('rgb/preview/image_raw', '/color/image'),
                                    ],
                    ),
            ],
        ),

    ]


def generate_launch_description():
    package_prefix = get_package_share_directory("ros_coneslayer")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
