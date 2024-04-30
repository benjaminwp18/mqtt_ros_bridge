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
    config = os.path.join(
        get_package_share_directory('mqtt_ros_bridge'),
        'config',
        'lampi.yaml'
        )

    run_bridge_node = Node(
        package='mqtt_ros_bridge',
        executable='bridge_node',
        emulate_tty=True,
        output='screen',
        arguments=[config]
    )

    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        run_bridge_node,
        rqt
    ])
