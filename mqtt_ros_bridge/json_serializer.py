import json
from array import array
from typing import (Iterable, MutableSequence, Type, TypeAlias,
                    TypeVar, cast)

from numpy import floating, integer, ndarray
from numpy.typing import NDArray
from rclpy.type_support import check_is_valid_msg_type

from mqtt_ros_bridge.msg_typing import MsgLike, MsgLikeT


NestedDictionary: TypeAlias = dict[str, object]
ArrayElementT = TypeVar('ArrayElementT', int, float, str)

RESERVED_FIELD_TYPE = '/'
ENCODING = 'latin-1'  # Arbitrary encoding for decoding; latin-1 supports all byte values


def numpy_to_list(array_arg: NDArray[integer] | NDArray[floating]) -> list[int] | list[float]:
    """
    Convert a Numpy array to a list for easy serialization.

    Parameters
    ----------
    array_arg : NDArray[integer] | NDArray[floating]
        the Numpy array to convert

    Returns
    -------
    list[int] | list[float]
        the resulting list

    """
    if isinstance(array_arg[0], integer):
        return [int(x) for x in array_arg]

    return [float(x) for x in array_arg]


def array_to_list(array_arg: MutableSequence[ArrayElementT]) -> list[ArrayElementT]:
    """
    Convert a Python array to a list for easy serialization.

    Parameters
    ----------
    array_arg : MutableSequence[ArrayElementT]
        the array to convert

    Returns
    -------
    list[ArrayElementT]
        the resulting list

    """
    if len(array_arg) == 0:
        return []
    element_type = type(array_arg[0])
    return [element_type(x) for x in array_arg]


def json_serialize(msg: MsgLike) -> bytes:
    """
    Serialize the provided ROS message to a JSON string as bytes.

    Parameters
    ----------
    msg : MsgLike
        the message to serialize

    Returns
    -------
    bytes
        the serialized message

    """
    check_is_valid_msg_type(type(msg))

    msg_dict = json_serialize_recursive(msg)
    return json.dumps(msg_dict).encode()


def json_serialize_recursive(msg: MsgLike) -> NestedDictionary:
    """
    Recursively convert the fields of the provided message to a dictionary for conversion to JSON.

    Parameters
    ----------
    msg : MsgLike
        the message to convert

    Returns
    -------
    NestedDictionary
        the resulting dictionary

    """
    msg_dict = {}

    msg_fields_and_field_types = type(msg).get_fields_and_field_types()
    for field, field_types in msg_fields_and_field_types.items():
        value = getattr(msg, field)

        if isinstance(value, bytes):
            value = value.decode(ENCODING)
        elif isinstance(value, list) and len(value) > 0:
            if isinstance(value[0], bytes):
                value = cast(list[bytes], value)
                value = [byte.decode(ENCODING) for byte in value]
            elif RESERVED_FIELD_TYPE in field_types:
                value = [json_serialize_recursive(msg_in_list) for msg_in_list in value]
        elif isinstance(value, list) and len(value) == 0:
            value = []
        elif isinstance(value, ndarray):
            value = numpy_to_list(value)
        elif isinstance(value, array):
            value = array_to_list(value)
        elif RESERVED_FIELD_TYPE in field_types:
            value = json_serialize_recursive(value)
        msg_dict[field] = value

    return msg_dict


def json_deserialize(byte_msg: bytes, msg_type: Type[MsgLikeT]) -> MsgLikeT:
    """
    Deserialize the provided bytes (formatted as a JSON string) into the provided ROS message type.

    Parameters
    ----------
    byte_msg : bytes
        the bytes of the serialized message, formatted in JSON
    msg_type : Type[MsgLikeT]
        the type of the resulting ROS message

    Returns
    -------
    MsgLikeT
        a ROS message of the provided type, using the data fro mthe provided serialized message

    """
    check_is_valid_msg_type(msg_type)

    str_msg = byte_msg.decode()
    msg_dict = json.loads(str_msg)
    return json_deserialize_recursive(msg_dict, msg_type)


def json_deserialize_recursive(msg_dict: NestedDictionary,
                               msg_type: Type[MsgLikeT]) -> MsgLikeT:
    """
    Recusively convert the fields of the provided dictionary to the provided ROS message type.

    Parameters
    ----------
    msg_dict : NestedDictionary
        the dictionary to read
    msg_type : Type[MsgLikeT]
        the message type to create

    Returns
    -------
    MsgLikeT
        the resulting ROS message

    """
    msg = msg_type()
    set_value: object
    for field, value in msg_dict.items():
        field_default = getattr(msg, field)
        if isinstance(field_default, bytes):
            if isinstance(value, str):
                set_value = value.encode(ENCODING)
        elif isinstance(value, dict):
            set_value = json_deserialize_recursive(value, type(getattr(msg, field)))
        elif isinstance(field_default, list):
            if len(field_default) == 0:
                set_value = []
            else:
                field_default_element = field_default[0]
                if isinstance(field_default_element, bytes):
                    value = cast(list[str], value)
                    set_value = [byte.encode(ENCODING) for byte in value]
                elif RESERVED_FIELD_TYPE in msg_type.get_fields_and_field_types()[field]:
                    value = cast(Iterable[NestedDictionary], value)
                    set_value = [json_deserialize_recursive(msg_in_list,
                                                            type(getattr(msg, field)[0]))
                                 for msg_in_list in value]
                else:
                    set_value = value
        else:
            set_value = value

        setattr(msg, field, set_value)
    return msg
