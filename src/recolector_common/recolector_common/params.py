"""
Dataclass-driven ROS parameter helpers. Pure logic, no rclpy.

A parameter is named once, as a dataclass field. The same dataclass gives
the declared defaults and receives the values read back from the node, so
names cannot drift between `declare_parameter` and `get_parameter` calls.
The `node` argument only needs `declare_parameter` and `get_parameter`.

Grouped parameters (`camera.hfov_rad`, `filter.min_area_px`) use the optional
`prefix`: the ROS name is `prefix + field name`.
"""

from dataclasses import fields, is_dataclass
from typing import Any


def declare_params(node: Any, defaults: Any, prefix: str = "") -> None:
    """
    Declare one ROS parameter per field of the `defaults` dataclass.

    The parameter name is `prefix` plus the field name, and the field value
    is its default.
    """
    if not is_dataclass(defaults):
        raise TypeError(f"expected a dataclass instance, got {defaults!r}")
    for field in fields(defaults):
        node.declare_parameter(
            f"{prefix}{field.name}",
            getattr(defaults, field.name),
        )


def read_params[ParamsT](
    node: Any,
    defaults: ParamsT,
    prefix: str = "",
) -> ParamsT:
    """
    Build a new dataclass of the same type as `defaults` from the node's
    current parameter values.

    Values are coerced to the type of the default (an int set from YAML into
    a float field becomes a float, a sequence into a list).
    """
    values = {}
    for field in fields(defaults):
        default = getattr(defaults, field.name)
        value = node.get_parameter(f"{prefix}{field.name}").value
        values[field.name] = _coerce(value, default)
    return type(defaults)(**values)


def _coerce(value: Any, default: Any) -> Any:
    """
    Convert `value` to the type of `default` when that is safe to do.
    """
    if isinstance(default, bool):
        return bool(value)
    if isinstance(default, float):
        return float(value)
    if isinstance(default, list):
        return list(value)
    return value
