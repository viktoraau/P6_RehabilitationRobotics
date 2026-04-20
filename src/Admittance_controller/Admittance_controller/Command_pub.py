import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
import numpy as np

class CommandPub(Node):
    def __init__(self):
        super().__init__('command_pub')
        
        self.subscriptionTraj = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.listener_traj_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/joint_group_velocity_controller/commands',
            10)
        
        
        self.commands = [2,0,1]
        self.joint_speeds = np.zeros(3)
        self.joints = np.full((3,4), -1)


    
    def listener_traj_callback(self, msg):

        self.joint_speeds = [msg.points[0].velocities[0],
                            msg.points[0].velocities[1], 
                            msg.points[0].velocities[2]]
        #self.get_logger().info(f"Joint speeds: {self.joint_speeds}")

        speed = Float64MultiArray()
        speed.data = np.zeros(3)
        speed.data[self.commands[0]] = self.joint_speeds[0]
        speed.data[self.commands[1]] = self.joint_speeds[1] * 4
        speed.data[self.commands[2]] = self.joint_speeds[2] * 3

        self.publisher_.publish(speed)
        #self.get_logger().info(f"Published speeds: {speed.data}")

        




def main(args=None):
    rclpy.init(args=args)
    node = CommandPub()

    rclpy.spin(node)   

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()