from typing import Any
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from std_msgs.msg import String

import paho.mqtt.client as MQTT


@dataclass
class TopicInfo():
    """Metadata about a single topic."""

    name: str
    publish_on_ros: bool


TOPICS: list[TopicInfo] = [
    TopicInfo('pub_topic', True),
    TopicInfo('sub_topic', False)
]


class BridgeNode(Node):
    """Node to bridge MQTT and ROS."""

    def __init__(self) -> None:
        super().__init__('mqtt_bridge_node')

        print('Creating MQTT ROS bridge node')

        self.mqtt_client = MQTT.Client()
        self.mqtt_client.enable_logger()
        self.mqtt_client.connect('localhost', port=1883, keepalive=60)
        self.mqtt_client.loop_start()

        self.ros_publishers: dict[str, Publisher] = {}
        self.ros_subscriptions: list[Subscription] = []

        for topic_info in TOPICS:
            if topic_info.publish_on_ros:
                publisher = self.create_publisher(String, topic_info.name, 10)
                self.ros_publishers[topic_info.name] = publisher
                self.mqtt_client.subscribe(topic_info.name)
            else:
                subscription = self.create_subscription(
                    String, topic_info.name, self.make_ros_receiver(topic_info.name), 10)
                self.ros_subscriptions.append(subscription)

        self.mqtt_client.on_message = self.mqtt_msg_received

    def make_ros_receiver(self, topic: str):
        """
        Create a callback function which re-publishes messages on the same topic in MQTT.

        Parameters
        ----------
        topic : str
            the topic that the callback will publish on

        """
        def callback(msg: String):
            self.get_logger().info(f"ROS RECEIVED: Topic: '{topic}' Payload: '{msg}'")
            self.mqtt_client.publish(topic, msg.data)

        return callback

    def mqtt_msg_received(self, _client: MQTT.Client, _userdata: Any, mqtt_msg: MQTT.MQTTMessage):
        """
        Re-publish messages from MQTT on the same topic in ROS.

        Parameters
        ----------
        _client : MQTT.Client
            unused; the MQTT client which received this message
        _userdata : Any
            unused; the private user data as set for the client
        mqtt_msg : MQTT.MQTTMessage
            the message received over MQTT

        """
        self.get_logger().info(
            f"MQTT RECEIVED: Topic: '{mqtt_msg.topic}' Payload: '{mqtt_msg.payload!r}'")

        ros_msg = String()
        ros_msg.data = mqtt_msg.payload.decode('utf-8')
        self.ros_publishers[mqtt_msg.topic].publish(ros_msg)


def main(args=None):
    """Run bridge node; used in ROS executable."""
    rclpy.init(args=args)

    bridge_node = BridgeNode()

    rclpy.spin(bridge_node)

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
