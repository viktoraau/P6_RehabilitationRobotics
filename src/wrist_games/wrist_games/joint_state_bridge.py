import threading
from math import tanh
from typing import Optional, Sequence, Tuple

from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateBridge(Node):
    def __init__(self, topic_name: str = "/joint_states", joint_names: Optional[Sequence[str]] = None) -> None:
        super().__init__("wrist_joint_state_bridge")
        self._joint_names = list(joint_names) if joint_names else []
        self._latest = [0.0, 0.0, 0.0]
        self._lock = threading.Lock()
        self.create_subscription(JointState, topic_name, self._on_joint_state, 20)

    def _on_joint_state(self, msg: JointState) -> None:
        positions = [0.0, 0.0, 0.0]
        if self._joint_names and msg.name:
            name_to_index = {name: i for i, name in enumerate(msg.name)}
            for i, joint_name in enumerate(self._joint_names[:3]):
                idx = name_to_index.get(joint_name)
                if idx is not None and idx < len(msg.position):
                    positions[i] = float(msg.position[idx])
        else:
            for i in range(min(3, len(msg.position))):
                positions[i] = float(msg.position[i])
        with self._lock:
            self._latest = positions

    def get_positions(self) -> Tuple[float, float, float]:
        with self._lock:
            return (self._latest[0], self._latest[1], self._latest[2])

    def get_normalized(self, gain: float = 1.0) -> Tuple[float, float, float]:
        raw = self.get_positions()
        return (
            max(-1.0, min(1.0, tanh(raw[0] * gain))),
            max(-1.0, min(1.0, tanh(raw[1] * gain))),
            max(-1.0, min(1.0, tanh(raw[2] * gain))),
        )
