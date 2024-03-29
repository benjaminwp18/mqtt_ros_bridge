import json
from array import array
from typing import (Iterable, MutableSequence, Protocol, Type, TypeAlias,
                    TypeVar, cast)

from numpy import ndarray, floating, integer
from numpy.typing import NDArray
from rclpy.type_support import check_is_valid_msg_type

NestedDictionary: TypeAlias = dict[str, object]


class MsgLike(Protocol):
    """Generic Message Type Alias."""

    @classmethod
    def get_fields_and_field_types(cls) -> dict[str, str]:
        ...


MsgLikeT = TypeVar("MsgLikeT", bound=MsgLike)
ArrayElementT = TypeVar('ArrayElementT', int, float, str)

RESERVED_FIELD_TYPE = '/'
ENCODING = 'latin-1'


def numpy_encoding(array_arg: NDArray[integer] | NDArray[floating]) -> list[int] | list[float]:
    if isinstance(array_arg[0], integer):
        return [int(x) for x in array_arg]
    else:
        return [float(x) for x in array_arg]


def array_encoding(array_arg: MutableSequence[ArrayElementT]) -> list[ArrayElementT]:
    if len(array_arg) == 0:
        return []
    element_type = type(array_arg[0])
    return [element_type(x) for x in array_arg]


def human_readable_encoding(msg: MsgLike) -> bytes:
    check_is_valid_msg_type(type(msg))

    msg_dict = human_readable_encoding_recursive(msg)
    return json.dumps(msg_dict).encode()


def human_readable_encoding_recursive(msg: MsgLike) -> NestedDictionary:
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
                value = [human_readable_encoding_recursive(msg_in_list) for msg_in_list in value]
        elif isinstance(value, list) and len(value) == 0:
            value = []
        elif isinstance(value, ndarray):
            value = numpy_encoding(value)
        elif isinstance(value, array):
            value = array_encoding(value)
        elif RESERVED_FIELD_TYPE in field_types:
            value = human_readable_encoding_recursive(value)
        msg_dict[field] = value

    return msg_dict


def human_readable_decoding(byte_msg: bytes, msg_type: Type[MsgLikeT]) -> MsgLikeT:
    check_is_valid_msg_type(msg_type)

    str_msg = byte_msg.decode()
    msg_dict = json.loads(str_msg)
    return human_readable_decoding_recursive(msg_dict, msg_type)


def human_readable_decoding_recursive(msg_dict: NestedDictionary,
                                      msg_type: Type[MsgLikeT]) -> MsgLikeT:
    msg = msg_type()
    set_value: object
    for field, value in msg_dict.items():
        field_default = getattr(msg, field)
        if isinstance(field_default, bytes):
            if isinstance(value, str):
                set_value = value.encode(ENCODING)
        elif isinstance(value, dict):
            set_value = human_readable_decoding_recursive(value, type(getattr(msg, field)))
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
                    set_value = [human_readable_decoding_recursive(msg_in_list,
                                                                   type(getattr(msg, field)[0]))
                                 for msg_in_list in value]
                else:
                    set_value = value
        else:
            set_value = value

        setattr(msg, field, set_value)
    return msg
