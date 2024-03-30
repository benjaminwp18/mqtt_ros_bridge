from typing import Protocol, TypeVar


class MsgLike(Protocol):
    """Generic Message Type Alias."""

    @classmethod
    def get_fields_and_field_types(cls) -> dict[str, str]:
        ...


MsgLikeT = TypeVar("MsgLikeT", bound=MsgLike)
