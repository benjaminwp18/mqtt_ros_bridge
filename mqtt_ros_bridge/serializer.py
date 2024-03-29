from abc import ABC, abstractmethod
from typing import Type

from mqtt_ros_bridge.encodings import (MsgLike, MsgLikeT,
                                       human_readable_decoding,
                                       human_readable_encoding)
from rclpy.serialization import deserialize_message, serialize_message


class Serializer(ABC):
    """Serializes and deserializes ROS messages for transmission over MQTT."""

    @staticmethod
    @abstractmethod
    def serialize(message: MsgLike) -> bytes:
        """
        Serialize the provided ROS message to a bytes for MQTT.

        Parameters
        ----------
        message : Any
            the ROS message to serialize

        Returns
        -------
        bytes
            the bytes formed by serializing the ROS message

        """

    @staticmethod
    @abstractmethod
    def deserialize(serialized_message: bytes, message_type: Type[MsgLikeT]) -> MsgLikeT:
        """
        Deserialize the provided bytes into a ROS message of the provided type.

        Parameters
        ----------
        serialized_message : bytes
            the bytes generated by serializing a ROS message
        message_type : Any
            the type of the message to create

        Returns
        -------
        Any
            the ROS message formed by deserializing the bytes

        """


class ROSDefaultSerializer(Serializer):
    """Serialize and deserialize messages using the default ROS message serializer."""

    @staticmethod
    def serialize(message: MsgLike) -> bytes:
        return serialize_message(message)

    @staticmethod
    def deserialize(serialized_message: bytes, message_type: Type[MsgLikeT]) -> MsgLikeT:
        return deserialize_message(serialized_message, message_type)


class HumanReadableSerializer(Serializer):
    """Serialize and deserialize messages using the default ROS message serializer."""

    @staticmethod
    def serialize(message: MsgLike) -> bytes:
        return human_readable_encoding(message)

    @staticmethod
    def deserialize(serialized_message: bytes, message_type: Type[MsgLikeT]) -> MsgLikeT:
        return human_readable_decoding(serialized_message, message_type)