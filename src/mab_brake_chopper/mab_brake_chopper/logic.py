from typing import Optional

from control_msgs.msg import DynamicJointState


class BrakeChopperController:
    """Simple threshold controller: HIGH when voltage < trigger, LOW when over."""

    def __init__(self, trigger_voltage_v: float) -> None:
        if trigger_voltage_v <= 0.0:
            raise ValueError("trigger_voltage_v must be positive")

        self.trigger_voltage_v = float(trigger_voltage_v)
        self.enabled = False

    def update(self, voltage_v: float) -> bool:
        next_state = voltage_v < self.trigger_voltage_v
        changed = next_state != self.enabled
        self.enabled = next_state
        return changed

    def force_disable(self) -> bool:
        changed = self.enabled
        self.enabled = False
        return changed


def extract_dynamic_joint_state_value(
    msg: DynamicJointState, joint_name: str, interface_name: str
) -> Optional[float]:
    for current_name, interface_values in zip(msg.joint_names, msg.interface_values):
        if current_name != joint_name:
            continue

        for current_interface, value in zip(
            interface_values.interface_names, interface_values.values
        ):
            if current_interface == interface_name:
                return float(value)

    return None
