from test_msgs.msg import (Arrays, BasicTypes, BoundedPlainSequences, BoundedSequences, Builtins,
                           Constants, Defaults, Empty, MultiNested, Strings, UnboundedSequences,
                           WStrings)

from mqtt_ros_bridge.msg_typing import MsgLikeT
from mqtt_ros_bridge.serializer import Serializer, ROSDefaultSerializer, JSONSerializer


messages = [
    Arrays(),
    BasicTypes(),
    BoundedPlainSequences(),
    BoundedSequences(),
    Builtins(),
    Constants(),
    Defaults(),
    Empty(),
    MultiNested(),
    Strings(),
    UnboundedSequences(),
    WStrings()
]


def serialize_checker(msg: MsgLikeT, serializer: type[Serializer]) -> MsgLikeT:
    """
    Check that a message serializes and deserializes to itself.

    Parameters
    ----------
    msg : MsgLikeT
        the message to check
    serializer : type[Serializer]
        the serializer to use

    Returns
    -------
    MsgLikeT
        the deserialized message

    """
    encoded_and_decoded_msg = serializer.deserialize(serializer.serialize(msg), type(msg))
    assert encoded_and_decoded_msg == msg
    return encoded_and_decoded_msg


def test_serializers() -> None:
    """Test that serialization works."""
    for message in messages:
        serialize_checker(message, ROSDefaultSerializer)
        serialize_checker(message, JSONSerializer)
