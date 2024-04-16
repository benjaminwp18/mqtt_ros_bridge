from importlib import import_module


def lookup_object(object_path: str, package: str = 'mqtt_ros_bridge') -> object:
    """ lookup object from a some.module:object_name specification. """
    module_name, obj_name = object_path.split(":")
    module = import_module(module_name, package)
    obj = getattr(module, obj_name)
    return obj
