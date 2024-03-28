from typing import Protocol, Type, TypeVar, TypeAlias, cast, Iterable

from numpy import ndarray, array
from numpy.typing import NDArray

import json
from rclpy.type_support import check_is_valid_msg_type


NestedDictionary: TypeAlias = dict[str, 'NestedDictionary'] | dict[str, object]


class MsgLike(Protocol):
    """Generic Message Type Alias."""

    @classmethod
    def get_fields_and_field_types(cls) -> dict[str, str]:
        ...


MsgLikeT = TypeVar("MsgLikeT", bound=MsgLike)


RESERVED_FIELD_TYPE = '_msgs/'
ENCODING = 'latin-1'


def numpy_encoding(array: NDArray) -> list:
    return [int(x) for x in array]


def numpy_decoding(ls: list) -> NDArray:
    return array(ls)


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
        elif isinstance(value, ndarray):
            value = numpy_encoding(value)
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
        if isinstance(getattr(msg, field), bytes):
            if isinstance(value, str):
                set_value = value.encode(ENCODING)
        elif isinstance(value, dict):
            set_value = human_readable_decoding_recursive(value, type(getattr(msg, field)))
        else:
            set_value = value

        setattr(msg, field, set_value)
    return msg

from test_msgs.msg import Arrays

# print(Arrays())

print(human_readable_decoding(human_readable_encoding(Arrays()), Arrays))
