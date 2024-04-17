import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate LaunchDescription for MQTT ROS bridge.

    Returns
    -------
    LaunchDescription
        Launches bridge_node.

    """
    turtle_sim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "2"),
        turtle_sim
    ])
