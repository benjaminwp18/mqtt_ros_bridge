import os
import sys
from typing import Any, Callable, Generic, cast
# from queue import queue
from enum import Enum

import paho.mqtt.client as MQTT
import rclpy
from rclpy._rclpy_pybind11 import RMWError
from rclpy.node import Node
from rclpy.client import Client
# TODO this breaks humble
from rclpy.parameter import Parameter, parameter_dict_from_yaml_file
from rclpy.publisher import Publisher

from mqtt_ros_bridge.msg_typing import MsgLikeT
from mqtt_ros_bridge.serializer import JSONSerializer, ROSDefaultSerializer
from mqtt_ros_bridge.util import lookup_message


class InteractionType(Enum):
    ROS_PUBLISHER = 1
    ROS_SUBSCRIBER = 2
    ROS_CLIENT = 3
    ROS_SERVER = 4


INTERACTION_TYPES: dict[str, InteractionType] = {
    'ros_publisher': InteractionType.ROS_PUBLISHER,
    'ros_subscriber': InteractionType.ROS_SUBSCRIBER,
    'ros_client': InteractionType.ROS_CLIENT,
    'ros_server': InteractionType.ROS_SERVER
}


class TopicMsgInfo(Generic[MsgLikeT]):
    """Metadata about a single topic."""

    def __init__(self, topic: str, msg_object_path: str, interaction_type: InteractionType,
                 use_ros_serializer: bool = True) -> None:

        self.topic = topic
        self.msg_type: MsgLikeT = cast(MsgLikeT, lookup_message(msg_object_path))
        self.interaction_type = interaction_type
        self.serializer = ROSDefaultSerializer if use_ros_serializer else JSONSerializer

    def __str__(self) -> str:
        return (f"Topic: {self.topic}, Message Type: {self.msg_type}, Type: "
                f"{self.interaction_type}, Serializer: {self.serializer}")


MQTT_PORT = 1883
MQTT_KEEPALIVE = 60

PARAMETER_TOPIC = "topic"
PARAMETER_TYPE = "type"
PARAMETER_INTERACTION_TYPE = "interaction_type"
PARAMETER_USE_ROS_SERIALIZER = "use_ros_serializer"

SERVICE_REQUEST_POSTFIX = "_mqtt_bridge_request"
SERVICE_RESPONSE_POSTFIX = "_mqtt_bridge_response"


class BridgeNode(Node):
    """Node to bridge MQTT and ROS."""

    def __init__(self, args: list[str]) -> None:
        super().__init__('mqtt_bridge_node')

        # TODO get from parameters
        # DEBUG = True

        print(args)

        if (len(args) != 2 and "--ros-args" not in args) or not (len(args) == 3 and
                                                                 "--ros-args" in args):
            msg = "To many arguments given"
            self.get_logger().error(msg)
            raise ValueError(msg)

        self.topics = self.topic_info_from_parameters(args[1])

        for topic, topic_info in self.topics.items():
            print(f'{topic}: {str(topic_info)}')

        self.get_logger().info('Creating MQTT ROS bridge node')

        self.mqtt_client = MQTT.Client()
        self.mqtt_client.enable_logger()
        self.mqtt_client.connect('localhost', port=MQTT_PORT, keepalive=MQTT_KEEPALIVE)
        self.mqtt_client.loop_start()

        self.ros_publishers: dict[str, Publisher] = {}
        self.ros_clients: dict[str, Client] = {}

        for topic_info in self.topics.values():
            match topic_info.interaction_type:
                case InteractionType.ROS_PUBLISHER:
                    publisher = self.create_publisher(topic_info.msg_type, topic_info.topic, 10)
                    self.ros_publishers[topic_info.topic] = publisher
                    self.mqtt_client.subscribe(topic_info.topic)
                case InteractionType.ROS_SUBSCRIBER:
                    callback = self.make_ros_callback(topic_info)
                    # TODO proper QOS?
                    self.create_subscription(topic_info.msg_type, topic_info.topic, callback, 10)
                case InteractionType.ROS_CLIENT:
                    self.ros_clients[topic_info.topic] = \
                        self.create_client(topic_info.topic, topic_info.topic)
                case InteractionType.ROS_SERVER:
                    pass  # TODO
                case _:
                    msg = f'Invalid interaction type {topic_info.interaction_type}'
                    self.get_logger().error(msg)
                    raise ValueError(msg)

        self.mqtt_client.on_message = self.mqtt_callback

    def topic_info_from_parameters(self, config: str) -> dict[str, TopicMsgInfo]:
        """Take a path to a config file and returns a TopicMsgInfo dictionary."""
        config = os.path.expanduser(config)
        topic_infos: dict[str, TopicMsgInfo] = {}

        params: dict[str, Parameter] = {}
        dictionary = parameter_dict_from_yaml_file(config)

        for key, parameter_msg in dictionary.items():
            params[key] = Parameter.from_parameter_msg(parameter_msg)

        unique_names: set[str] = set()
        for names in params.keys():
            # TODO Check that right half matches PARAMETER_*
            unique_names.add(names.split(".")[0])

        for name in unique_names:
            if params.get(f"{name}.{PARAMETER_USE_ROS_SERIALIZER}", None):
                ros_serialiser = params[f"{name}.{PARAMETER_USE_ROS_SERIALIZER}"].value
            else:
                ros_serialiser = False

            interaction_type_str = params[f"{name}.{PARAMETER_INTERACTION_TYPE}"].value
            if interaction_type_str not in INTERACTION_TYPES.keys():
                msg = f'Interaction types must be one of {INTERACTION_TYPES.keys()}'
                self.get_logger().error(msg)
                raise ValueError(msg)

            topic_infos[params[f"{name}.{PARAMETER_TOPIC}"].value] = TopicMsgInfo(
                params[f"{name}.{PARAMETER_TOPIC}"].value,
                params[f"{name}.{PARAMETER_TYPE}"].value,
                INTERACTION_TYPES[interaction_type_str],
                ros_serialiser
            )

        return topic_infos

    def make_ros_callback(self, topic_info: TopicMsgInfo[MsgLikeT]) -> Callable[[MsgLikeT], None]:
        """
        Create a callback function which re-publishes messages on the same topic in MQTT.

        Parameters
        ----------
        topic_info : TopicMsgInfo
            information about the topic that the callback will publish on

        """
        def callback(msg: MsgLikeT) -> None:
            self.get_logger().info(f'ROS RECEIVED: Topic: "{topic_info.topic}" Payload: "{msg}"')
            self.mqtt_client.publish(topic_info.topic, topic_info.serializer.serialize(msg))

        return callback

    def mqtt_callback(self, _client: MQTT.Client,
                      _userdata: Any, mqtt_msg: MQTT.MQTTMessage) -> None:
        """
        Re-publish messages from MQTT on the same topic in ROS or 

        Parameters
        ----------
        _client : MQTT.Client
            unused; the MQTT client which received this message
        _userdata : Any
            unused; the private user data as set for the client
        mqtt_msg : MQTT.MQTTMessage
            the message received over MQTT

        """
        if mqtt_msg.topic in self.topics:
            topic_info = self.topics[mqtt_msg.topic]
        elif mqtt_msg.topic + SERVICE_REQUEST_POSTFIX in self.topics:
            topic_info = self.topics[mqtt_msg.topic + SERVICE_REQUEST_POSTFIX]
        elif mqtt_msg.topic + SERVICE_RESPONSE_POSTFIX in self.topics:
            topic_info = self.topics[mqtt_msg.topic + SERVICE_RESPONSE_POSTFIX]
        else:
            return  # Ignore MQTT messages on topics that weren't registered with us

        self.get_logger().info(
            f'MQTT RECEIVED: Topic: "{topic_info.topic}" Payload: "{mqtt_msg.payload!r}"')

        try:
            ros_msg = topic_info.serializer.deserialize(mqtt_msg.payload, topic_info.msg_type)
        except RMWError:
            self.get_logger().info('Dropping message with bad serialization received from' +
                                   f'MQTT on topic "{mqtt_msg.topic}": "{mqtt_msg.payload!r}"')
            return

        if topic_info.interaction_type == InteractionType.ROS_PUBLISHER:
            self.ros_publishers[topic_info.topic].publish(ros_msg)
        elif topic_info.interaction_type == InteractionType.ROS_CLIENT:
            request = topic_info.msg_type.Request()
            future = self.ros_clients[topic_info.topic].call_async(request)
            # TODO: send a message on topic + RESPONSE POSTFIX when this future returns
        # TODO: server


def main(args=None):
    """Run bridge node; used in ROS executable."""
    rclpy.init(args=args)

    bridge_node = BridgeNode(sys.argv)

    rclpy.spin(bridge_node)

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
