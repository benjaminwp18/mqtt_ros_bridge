from typing import Protocol, Type, TypeVar, NoReturn

import json
from rclpy.type_support import check_is_valid_msg_type


NestedDictionary = dict[str, 'NestedDictionary'] | dict[str, str] | dict[str, NoReturn]


class MsgLike(Protocol):
    """Generic Message Type Alias."""

    @classmethod
    def get_fields_and_field_types(cls) -> dict[str, str]:
        ...


MsgLikeT = TypeVar("MsgLikeT", bound=MsgLike)


RESERVED_FIELD_TYPE = '_msgs/'


def human_readable_encoding(msg: MsgLike) -> bytes:
    check_is_valid_msg_type(type(msg))
    return json.dumps(human_readable_encoding_recursive(msg)).encode()


def human_readable_encoding_recursive(msg: MsgLike) -> NestedDictionary:
    msg_dict: NestedDictionary = {}
    for field, field_types in (msg.get_fields_and_field_types()).items():
        value = getattr(msg, field)

        if isinstance(value, bytes):
            value = value.decode()
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

    for field, value in msg_dict.items():
        if isinstance(getattr(msg, field), bytes):
            if isinstance(value, str):
                value = value.encode()
        elif isinstance(value, dict):
            value = human_readable_decoding_recursive(value, type(getattr(msg, field)))

        setattr(msg, field, value)
    return msg
