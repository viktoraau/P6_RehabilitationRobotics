import rclpy
import time as t
from gpiozero import OutputDevice
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
import math

relay = OutputDevice(17, active_high=False, initial_value=False)

measurements = []
newest_measurements = []


class BrakeRelay(Node):
    def __init__(self):
        self.relay_delay = t.time() - 0.5
        super().__init__('brake_relay')

        self.subscription = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        voltages = []


        for i, joint_name in enumerate(msg.joint_names):
            interfaces = msg.interface_values[i]
        
            if "voltage" in interfaces.interface_names:
                idx = interfaces.interface_names.index("voltage")
                val = interfaces.values[idx]

                if not math.isnan(val):
                    voltages.append(val)


        if len(voltages) == 0:
            self.get_logger().warn("No valid voltage readings")
            return

        val = sum(voltages) / len(voltages) 


        #self.get_logger().info(f"Avg voltage: {val:.3f}, Measured voltages: {len(voltages)}")


        if len(measurements) >= 100:
            measurements.pop(0)
        if len(newest_measurements) >= 10:
            newest_measurements.pop(0)

        measurements.append(val)
        newest_measurements.append(val)

        avg = sum(measurements) / len(measurements)
        val_avg = sum(newest_measurements) / len(newest_measurements)


        if val_avg > 12 and not relay.is_active and t.time() > self.relay_delay + 0.5:
            relay.on()
            self.relay_delay = t.time()
            self.get_logger().info("Relay is ON")

        if val_avg <= 12 and relay.is_active and t.time() > self.relay_delay + 0.3:
            relay.off()
            self.get_logger().info("Relay is OFF")


def main(args=None):
    rclpy.init(args=args)

    node = BrakeRelay()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()