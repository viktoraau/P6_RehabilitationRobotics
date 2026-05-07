from typing import Iterable, Optional

from control_msgs.msg import DynamicJointState


class BrakeChopperController:
    """Simple low-voltage controller: HIGH when voltage drops below trigger."""

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


def extract_dynamic_joint_state_values(
    msg: DynamicJointState, joint_names: Iterable[str], interface_name: str
) -> list[float]:
    wanted_names = {name.strip() for name in joint_names if name and name.strip()}
    if not wanted_names:
        return []

    values = []
    for current_name, interface_values in zip(msg.joint_names, msg.interface_values):
        if current_name not in wanted_names:
            continue

        for current_interface, value in zip(
            interface_values.interface_names, interface_values.values
        ):
            if current_interface == interface_name:
                values.append(float(value))
                break

    return values
