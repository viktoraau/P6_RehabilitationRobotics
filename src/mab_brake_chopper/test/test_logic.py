from control_msgs.msg import DynamicJointState, InterfaceValue

from mab_brake_chopper.logic import BrakeChopperController, extract_dynamic_joint_state_value


def test_threshold_controller_turns_on_above_trigger_and_off_below():
    controller = BrakeChopperController(trigger_voltage_v=54.0)

    assert controller.enabled is False
    assert controller.update(54.5) is True
    assert controller.enabled is True

    assert controller.update(55.0) is False
    assert controller.enabled is True

    assert controller.update(53.8) is True
    assert controller.enabled is False


def test_extract_dynamic_joint_state_value_reads_matching_interface():
    msg = DynamicJointState()
    msg.joint_names = ["joint_1", "mab_power_stage"]
    msg.interface_values = [
        InterfaceValue(interface_names=["position"], values=[1.2]),
        InterfaceValue(interface_names=["bus_voltage", "temperature"], values=[53000.0, 44.0]),
    ]

    assert (
        extract_dynamic_joint_state_value(
            msg, joint_name="mab_power_stage", interface_name="bus_voltage"
        )
        == 53000.0
    )


def test_extract_dynamic_joint_state_value_returns_none_when_missing():
    msg = DynamicJointState()
    msg.joint_names = ["mab_power_stage"]
    msg.interface_values = [InterfaceValue(interface_names=["current"], values=[12.0])]

    assert (
        extract_dynamic_joint_state_value(
            msg, joint_name="mab_power_stage", interface_name="bus_voltage"
        )
        is None
    )
