from importlib import import_module
from typing import Optional

import yaml
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.parameter import PARAMETER_SEPARATOR_STRING, Parameter


def lookup_object(object_path: str, package: str = 'mqtt_ros_bridge') -> object:
    """ lookup object from a some.module:object_name specification. """
    module_name, obj_name = object_path.split(":")
    module = import_module(module_name, package)
    obj = getattr(module, obj_name)
    return obj


# Kind of exist in ros-iron??? very confusing
def get_parameter_value(string_value: str) -> ParameterValue:
    """
    Guess the desired type of the parameter based on the string value.

    :param string_value: The string value to be converted to a ParameterValue.
    :return: The ParameterValue.
    """
    value = ParameterValue()
    try:
        yaml_value = yaml.safe_load(string_value)
    except yaml.parser.ParserError:
        yaml_value = string_value

    if isinstance(yaml_value, bool):
        value.type = ParameterType.PARAMETER_BOOL
        value.bool_value = yaml_value
    elif isinstance(yaml_value, int):
        value.type = ParameterType.PARAMETER_INTEGER
        value.integer_value = yaml_value
    elif isinstance(yaml_value, float):
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = yaml_value
    elif isinstance(yaml_value, list):
        if all((isinstance(v, bool) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_BOOL_ARRAY
            value.bool_array_value = yaml_value
        elif all((isinstance(v, int) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_INTEGER_ARRAY
            value.integer_array_value = yaml_value
        elif all((isinstance(v, float) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            value.double_array_value = yaml_value
        elif all((isinstance(v, str) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_STRING_ARRAY
            value.string_array_value = yaml_value
        else:
            value.type = ParameterType.PARAMETER_STRING
            value.string_value = string_value
    else:
        value.type = ParameterType.PARAMETER_STRING
        value.string_value = yaml_value if yaml_value is not None else string_value
    return value


def parameter_value_to_python(parameter_value: ParameterValue):
    """
    Get the value for the Python builtin type from a rcl_interfaces/msg/ParameterValue object.

    Returns the value member of the message based on the ``type`` member.
    Returns ``None`` if the parameter is "NOT_SET".

    :param parameter_value: The message to get the value from.
    :raises RuntimeError: if the member ``type`` has an unexpected value.
    """
    if parameter_value.type == ParameterType.PARAMETER_BOOL:
        value = parameter_value.bool_value
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
        value = parameter_value.integer_value
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
        value = parameter_value.double_value
    elif parameter_value.type == ParameterType.PARAMETER_STRING:
        value = parameter_value.string_value
    elif parameter_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        value = list(parameter_value.byte_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
        value = list(parameter_value.bool_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        value = list(parameter_value.integer_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        value = list(parameter_value.double_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
        value = list(parameter_value.string_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_NOT_SET:
        value = None
    else:
        raise RuntimeError(f'unexpected parameter type {parameter_value.type}')

    return value


def parameter_dict_from_yaml_file(
    parameter_file: str,
    use_wildcard: bool = False,
    target_nodes: Optional[list[str]] = None,
    namespace: str = ''
) -> dict[str, Parameter]:
    """
    Build a dict of parameters from a YAML file.

    Will load all parameters if ``target_nodes`` is None or empty.

    :raises RuntimeError: if a target node is not in the file
    :raises RuntimeError: if the is not a valid ROS parameter file

    :param parameter_file: Path to the YAML file to load parameters from.
    :param use_wildcard: Use wildcard matching for the target nodes.
    :param target_nodes: list of nodes in the YAML file to load parameters from.
    :param namespace: Namespace to prepend to all parameters.
    :return: A dict of Parameter messages keyed by the parameter names
    """
    with open(parameter_file, 'r') as f:
        param_file = yaml.safe_load(f)
        param_keys = []
        param_dict = {}

        if use_wildcard and '/**' in param_file:
            param_keys.append('/**')

        if target_nodes:
            for n in target_nodes:
                if n not in param_file.keys():
                    raise RuntimeError(f'Param file does not contain parameters for {n},'
                                       f'only for nodes: {list(param_file.keys())} ')
                param_keys.append(n)
        else:
            # wildcard key must go to the front of param_keys so that
            # node-namespaced parameters will override the wildcard parameters
            keys = set(param_file.keys())
            keys.discard('/**')
            param_keys.extend(keys)

        if len(param_keys) == 0:
            raise RuntimeError('Param file does not contain selected parameters')

        for n in param_keys:
            value = param_file[n]
            if not isinstance(value, dict) or 'ros__parameters' not in value:
                raise RuntimeError(f'YAML file is not a valid ROS parameter file for node {n}')
            param_dict.update(value['ros__parameters'])
        # Modification done here
        new_dictionary: dict[str, Parameter] = {}
        dictionary = _unpack_parameter_dict(namespace, param_dict)

        for key, parameter_msg in dictionary.items():
            new_dictionary[key] = Parameter.from_parameter_msg(parameter_msg)
        return new_dictionary


def _unpack_parameter_dict(namespace, parameter_dict):
    """
    Flatten a parameter dictionary recursively.

    :param namespace: The namespace to prepend to the parameter names.
    :param parameter_dict: A dictionary of parameters keyed by the parameter names
    :return: A dict of Parameter objects keyed by the parameter names
    """
    parameters: dict[str, ParameterMsg] = {}
    for param_name, param_value in parameter_dict.items():
        full_param_name = namespace + param_name
        # Unroll nested parameters
        if isinstance(param_value, dict):
            parameters.update(_unpack_parameter_dict(
                    namespace=full_param_name + PARAMETER_SEPARATOR_STRING,
                    parameter_dict=param_value))
        else:
            parameter = ParameterMsg()
            parameter.name = full_param_name
            parameter.value = get_parameter_value(str(param_value))
            parameters[full_param_name] = parameter
    return parameters
