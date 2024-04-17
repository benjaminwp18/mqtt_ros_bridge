from importlib import import_module
from typing import cast

from mqtt_ros_bridge.msg_typing import MsgLike


def lookup_message(object_path: str, package: str = 'mqtt_ros_bridge') -> MsgLike:
    """Lookup message from a some.module:object_name specification."""
    return cast(MsgLike, _lookup_object(object_path, package))


def _lookup_object(object_path: str, package: str = 'mqtt_ros_bridge') -> object:
    """Lookup object from a some.module:object_name specification."""
    module_name, obj_name = object_path.split(":")
    module = import_module(module_name, package)
    obj = getattr(module, obj_name)
    return obj
