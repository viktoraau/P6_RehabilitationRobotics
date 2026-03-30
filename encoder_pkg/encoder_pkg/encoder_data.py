import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import numpy as np

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('EncoderReader started and subscribed to /joint_states')

    def joint_state_callback(self, msg: JointState):
        #self.get_logger().info('Received JointState message')

        if not msg.position:
            self.get_logger().warn('Message received, but position field is empty')
            return

        #for i, pos in enumerate(msg.position):
            #name = msg.name[i] if i < len(msg.name) else f'joint_{i}'
            #self.get_logger().info(f'{name}: {pos:.6f} rad')
        Position3 = (msg.position[2] * (180.0 / np.pi)) * 10
        Torque3 = (msg.effort[2])

        self.get_logger().info(f'Joint 3 position: {Position3} deg, Torque: {Torque3} Nm')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()