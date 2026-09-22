from dataclasses import fields
from drone_core.ros_support import parameter
from .battery import BatteryPolicy


def battery_policy(node) -> BatteryPolicy:
    defaults = BatteryPolicy()
    return BatteryPolicy(**{f.name: parameter(node, 'battery.' + f.name, getattr(defaults, f.name))
                            for f in fields(BatteryPolicy)})
