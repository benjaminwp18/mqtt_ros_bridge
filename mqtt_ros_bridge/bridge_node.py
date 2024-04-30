import os
import sys
from enum import IntEnum
from typing import Any, Callable, Generic, cast, Literal

import paho.mqtt.client as MQTT
import rclpy
from rclpy._rclpy_pybind11 import RMWError
from rclpy.node import Node
# TODO this breaks humble
from rclpy.qos import QoSPresetProfiles
from rclpy.parameter import Parameter, parameter_dict_from_yaml_file
from rclpy.publisher import Publisher

from mqtt_ros_bridge.msg_typing import MsgLikeT
from mqtt_ros_bridge.serializer import JSONSerializer, ROSDefaultSerializer
from mqtt_ros_bridge.util import lookup_message


class QoSPresetProfilesInt(IntEnum):
    UNKNOWN = 0
    DEFAULT = 1
    SYSTEM_DEFAULT = 2
    SENSOR_DATA = 3
    SERVICES_DEFAULT = 4
    PARAMETERS = 5
    PARAMETER_EVENTS = 6
    ACTION_STATUS_DEFAULT = 7
    BEST_AVAILABLE = 8

    @staticmethod
    def to_qos_profile(enum: int | 'QoSPresetProfilesInt') -> QoSPresetProfiles:
        match enum:
            case QoSPresetProfilesInt.UNKNOWN:
                return QoSPresetProfiles.UNKNOWN
            case QoSPresetProfilesInt.DEFAULT:
                return QoSPresetProfiles.DEFAULT
            case QoSPresetProfilesInt.SYSTEM_DEFAULT:
                return QoSPresetProfiles.SYSTEM_DEFAULT
            case QoSPresetProfilesInt.SENSOR_DATA:
                return QoSPresetProfiles.SENSOR_DATA
            case QoSPresetProfilesInt.SERVICES_DEFAULT:
                return QoSPresetProfiles.SERVICES_DEFAULT
            case QoSPresetProfilesInt.PARAMETERS:
                return QoSPresetProfiles.PARAMETERS
            case QoSPresetProfilesInt.PARAMETER_EVENTS:
                return QoSPresetProfiles.PARAMETER_EVENTS
            case QoSPresetProfilesInt.ACTION_STATUS_DEFAULT:
                return QoSPresetProfiles.ACTION_STATUS_DEFAULT
            case QoSPresetProfilesInt.BEST_AVAILABLE:
                return QoSPresetProfiles.BEST_AVAILABLE
            case _:
                return QoSPresetProfiles.UNKNOWN


class TopicMsgInfo(Generic[MsgLikeT]):
    """Metadata about a single topic."""

    def __init__(self, topic: str, msg_object_path: str,
                 publish_on_ros: bool, use_ros_serializer: bool = True,
                 ros_qos: int | QoSPresetProfilesInt = QoSPresetProfilesInt.DEFAULT,
                 mqtt_qos: Literal[0, 1, 2] = 0) -> None:

        self.topic = topic
        self.msg_type: MsgLikeT = cast(MsgLikeT, lookup_message(msg_object_path))
        self.publish_on_ros = publish_on_ros
        self.serializer = ROSDefaultSerializer if use_ros_serializer else JSONSerializer
        self.ros_qos = QoSPresetProfilesInt.to_qos_profile(ros_qos)
        self.mqtt_qos = mqtt_qos

    def __str__(self) -> str:
        return (f"Topic: {self.topic}, Message Type: {self.msg_type}, Publish on ROS:"
                f"{self.publish_on_ros}, Serializer: {self.serializer}")


MQTT_PORT = 50001
MQTT_KEEPALIVE = 60

PARAMETER_TOPIC = "topic"
PARAMETER_TYPE = "type"
PARAMETER_PUBLISH_ON_ROS = "publish_on_ros"
PARAMETER_USE_ROS_SERIALIZER = "use_ros_serializer"


class BridgeNode(Node):
    """Node to bridge MQTT and ROS."""

    def __init__(self, args: list[str]) -> None:
        super().__init__('mqtt_bridge_node')

        # TODO get from parameters
        # DEBUG = True

        print(args)

        if (len(args) != 2 and "--ros-args" not in args) or not (len(args) == 3 and
                                                                 "--ros-args" in args):
            raise ValueError("To many arguments given")

        self.topics = self.topic_info_from_parameters(args[1])

        self.get_logger().info(str(self.topics))

        self.get_logger().info('Creating MQTT ROS bridge node')

        self.mqtt_client = MQTT.Client()
        self.mqtt_client.enable_logger()
        self.mqtt_client.connect('localhost', port=MQTT_PORT, keepalive=MQTT_KEEPALIVE)
        self.mqtt_client.loop_start()

        self.ros_publishers: dict[str, Publisher] = {}

        for topic_info in self.topics.values():
            if topic_info.publish_on_ros:
                publisher = self.create_publisher(topic_info.msg_type, topic_info.topic, topic_info.ros_qos)
                self.ros_publishers[topic_info.topic] = publisher
                self.mqtt_client.subscribe(topic_info.topic, qos=topic_info.mqtt_qos)
            else:
                callback = self.make_ros_callback(topic_info)
                # TODO proper QOS?
                self.create_subscription(topic_info.msg_type, topic_info.topic, callback, topic_info.ros_qos)

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

            topic_infos[params[f"{name}.{PARAMETER_TOPIC}"].value] = (TopicMsgInfo(
                params[f"{name}.{PARAMETER_TOPIC}"].value,
                params[f"{name}.{PARAMETER_TYPE}"].value,
                params[f"{name}.{PARAMETER_PUBLISH_ON_ROS}"].value,
                ros_serialiser
            ))

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
            self.mqtt_client.publish(topic_info.topic, topic_info.serializer.serialize(msg), topic_info.mqtt_qos)

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
            f'MQTT RECEIVED: Topic: "{topic_info.topic}" Payload: "{mqtt_msg.payload!r}"')

        try:
            ros_msg = topic_info.serializer.deserialize(mqtt_msg.payload, topic_info.msg_type)
        except RMWError:
            self.get_logger().info('Dropping message with bad serialization received from' +
                                   f'MQTT on topic "{mqtt_msg.topic}": "{mqtt_msg.payload!r}"')
            return

        self.ros_publishers[topic_info.topic].publish(ros_msg)


def main(args=None):
    """Run bridge node; used in ROS executable."""
    rclpy.init(args=args)

    bridge_node = BridgeNode(sys.argv)

    rclpy.spin(bridge_node)

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
