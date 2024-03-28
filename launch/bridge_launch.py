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
    run_bridge_node = Node(
        package='mqtt_ros_bridge',
        executable='bridge_node',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        run_bridge_node
    ])
