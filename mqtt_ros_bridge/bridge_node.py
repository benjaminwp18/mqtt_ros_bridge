from typing import Any, Callable, Generic

import paho.mqtt.client as MQTT
import rclpy
from rclpy._rclpy_pybind11 import RMWError
from rclpy.node import Node
from rclpy.publisher import Publisher

from mqtt_ros_bridge.msg_typing import MsgLikeT
from mqtt_ros_bridge.serializer import (JSONSerializer, ROSDefaultSerializer,
                                        Serializer)
from mqtt_ros_bridge.util import lookup_object


class TopicInfo(Generic[MsgLikeT]):
    """Metadata about a single topic."""

    def __init__(self, name: str, msg_object_path: str,
                 publish_on_ros: bool, use_ros_serializer: bool = True) -> None:

        self.name = name
        self.msg_type = lookup_object(msg_object_path)
        self.publish_on_ros = publish_on_ros

        if use_ros_serializer:
            self.serializer: type[Serializer] = ROSDefaultSerializer
        else:
            self.serializer = JSONSerializer


# TOPICS: dict[str, TopicInfo] = {
#     '/turtle1/cmd_vel': TopicInfo('/turtle1/cmd_vel', "std_msgs.msg:String", False),
#     # 'sub_topic': TopicInfo('sub_topic', String, ROSDefaultSerializer, False)
# }

MQTT_PORT = 1883
MQTT_KEEPALIVE = 60

PARAMETER_TOPIC = "topic"
PARAMETER_TYPE = "type"
PARAMETER_PUBLISH_ON_ROS = "publish_on_ros"
PARAMETER_USE_ROS_SERIALIZER = "use_ros_serializer"


class BridgeNode(Node):
    """Node to bridge MQTT and ROS."""

    def __init__(self) -> None:
        super().__init__('mqtt_bridge_node')

        self.topics = self.topic_info_from_parameters()

        # TODO get from parameters
        DEBUG = True

        self.get_logger().info('Creating MQTT ROS bridge node')

        self.mqtt_client = MQTT.Client()
        self.mqtt_client.enable_logger()
        self.mqtt_client.connect('localhost', port=MQTT_PORT, keepalive=MQTT_KEEPALIVE)
        self.mqtt_client.loop_start()

        self.ros_publishers: dict[str, Publisher] = {}

        for topic_info in self.topics.values():
            if topic_info.publish_on_ros:
                publisher = self.create_publisher(topic_info.msg_type, topic_info.name, 10)
                self.ros_publishers[topic_info.name] = publisher
                self.mqtt_client.subscribe(topic_info.name)
            else:
                callback = self.make_ros_callback(topic_info)
                # TODO proper QOS?
                self.create_subscription(topic_info.msg_type, topic_info.name, callback, 10)

        self.mqtt_client.on_message = self.mqtt_callback

    def topic_info_from_parameters(self) -> dict[str, TopicInfo]:
        dictionary = {}

        params = self.get_parameters_by_prefix('')

        self.get_logger().info(str(params))

        return dictionary


    def make_ros_callback(self, topic_info: TopicInfo[MsgLikeT]) -> Callable[[MsgLikeT], None]:
        """
        Create a callback function which re-publishes messages on the same topic in MQTT.

        Parameters
        ----------
        topic_info : TopicInfo
            information about the topic that the callback will publish on

        """
        def callback(msg: MsgLikeT) -> None:
            self.get_logger().info(f'ROS RECEIVED: Topic: "{topic_info.name}" Payload: "{msg}"')
            self.mqtt_client.publish(topic_info.name, topic_info.serializer.serialize(msg))

        return callback

    def mqtt_callback(self, _client: MQTT.Client,
                      _userdata: Any, mqtt_msg: MQTT.MQTTMessage) -> None:
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
        topic_info = self.topics[mqtt_msg.topic]

        self.get_logger().info(
            f'MQTT RECEIVED: Topic: "{topic_info.name}" Payload: "{mqtt_msg.payload!r}"')

        try:
            ros_msg = topic_info.serializer.deserialize(mqtt_msg.payload, topic_info.msg_type)
        except RMWError:
            self.get_logger().info('Dropping message with bad serialization received from' +
                                   f'MQTT on topic "{mqtt_msg.topic}": "{mqtt_msg.payload!r}"')
            return

        self.ros_publishers[topic_info.name].publish(ros_msg)


def main(args=None):
    """Run bridge node; used in ROS executable."""
    rclpy.init(args=args)

    bridge_node = BridgeNode()

    rclpy.spin(bridge_node)

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
