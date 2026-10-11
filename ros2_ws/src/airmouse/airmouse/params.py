"""Parameter helper: dynamic typing so `7` vs `7.0` in YAML never crashes a node."""
from rcl_interfaces.msg import ParameterDescriptor


def declare(node, name, default):
    node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
    return node.get_parameter(name).value
