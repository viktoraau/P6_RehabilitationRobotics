import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray
import time as t
import numpy as np


class MotorCal(Node):
    def __init__(self):
        super().__init__('motor_cal')
        self.get_logger().info(f"test")
        self.current = t.time()
        self.previous_vel = 0
        self.ext = np.full((3,2), 99.0)
        self.motor_nr = 2
        self.stop = False
        self.joints = np.full((3,4), -1)
        self.commands = [2,0,1]
        self.torques = [0.9, 0.5, 0.02]
        self.speed_val = [1.5,5,3]
        self.stopped = False

        self.subscription = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.listener_callback,
            10)
        
        self.publisher_1 = self.create_publisher(
            Float64MultiArray,
            '/joint_group_velocity_controller/commands',
            10)
        
        self.publisher_2 = self.create_publisher(
            Float64MultiArray,
            '/Max_Min_Positions',
            10)

    # def listener_callback(self, msg):
    #     pos_val = msg.interface_values[0].values[2]
    #     eff_val = msg.interface_values[0].values[1]
    #     #self.get_logger().info(f"Position: {pos_val}, Effort: {eff_val}")
    #     self.effort_check(pos_val, eff_val)

    def listener_callback(self, msg):
            try:
                if self.joints[0,0] < 0 and self.joints[1,0] < 0 and self.joints[2,0] < 0:
                    self.get_indeces(msg)
                
                #self.get_indeces(msg)
                
                #self.get_logger().info(f"Joints: {self.joints}")
                vol_val = msg.interface_values[self.motor_nr].values[self.joints[self.motor_nr,3]]
                if np.isnan(vol_val):
                    self.get_logger().info(f"Motor number {self.motor_nr} is not found")
                    self.motor_nr = self.motor_nr - 1
                    return

                self.get_logger().info(f"Motor number: {self.motor_nr}, Extremities: {self.ext}")

                if self.motor_nr < 0:
                    self.motor_stop()
                    return

                pos_val = msg.interface_values[self.motor_nr].values[self.joints[self.motor_nr,0]]
                vel_val = msg.interface_values[self.motor_nr].values[self.joints[self.motor_nr,1]]
                eff_val = msg.interface_values[self.motor_nr].values[self.joints[self.motor_nr,2]]


                if self.ext[self.motor_nr,1] == 99:
                    self.effort_check(pos_val, eff_val, vel_val, self.motor_nr)
                elif self.ext[self.motor_nr,0] != 99 and self.ext[self.motor_nr,1] != 99:
                    self.middle_pos(pos_val, (self.ext[self.motor_nr,0] + self.ext[self.motor_nr,1]) / 2)

                # pos_val = msg.interface_values[1].values[2]
                # eff_val = msg.interface_values[1].values[0]
                # vel_val = msg.interface_values[1].values[1]
                #self.get_logger().info(f"Effort: {eff_val} Position: {pos_val}")
                # if self.stop:
                #     self.motor_stop()
                # elif self.ext[0] == 0:
                #     self.effort_check(pos_val, eff_val, vel_val, 0)
                # elif self.ext[1] == 0:
                #     self.effort_check(pos_val, eff_val, vel_val, 1)
                # elif self.ext[0] != 0 and self.ext[1] != 0:
                #     middle = (self.ext[0] + self.ext[1]) / 2
                #     self.middle_pos(pos_val, middle)
                
                
            except Exception as e:
                self.get_logger().error(f"Callback error: {e}")

    def effort_check(self, pos, eff, vel, i):
        speed = Float64MultiArray()
        if self.ext[i,0] != 99:
            idx = 1
        else:
            idx = 0
        
        self.get_logger().info(f"Torque limit: {self.torques[i]}")

        if abs(eff) > self.torques[i] and t.time() > self.current + 2 and self.previous_vel - abs(vel) > 0 and abs(vel) < 1.5:
            self.speed_val[i] = self.speed_val[i] * -1
            self.current = t.time()
            speed.data = np.zeros(3)
            while self.current + 0.2 > t.time():
                speed.data[self.commands[self.motor_nr]] = self.speed_val[i]
                self.publisher_1.publish(speed)
            #speed.data = [0,0,0]
            self.publisher_1.publish(speed)
            self.ext[i,idx] = pos
            self.get_logger().info(f"Position: {pos}")
            self.current = t.time()

        elif t.time() > self.current + 2:
            speed.data = np.zeros(3)
            speed.data[self.commands[self.motor_nr]] = self.speed_val[i]
            self.publisher_1.publish(speed)
        self.previous_vel = abs(vel)


    def middle_pos(self, pos, middle):
        error = middle - pos
        abs_error = abs(error)

        speed = Float64MultiArray()

        if abs_error < 0.05:
            speed.data = np.zeros(3)
            self.publisher_1.publish(speed)
            t.sleep(1)
            self.motor_nr = self.motor_nr - 1

        k = 4.0
        vel_cmd = k * error

        vel_cmd = max(min(vel_cmd, self.speed_val[self.motor_nr]), -self.speed_val[self.motor_nr])
        if 0 < vel_cmd < self.speed_val[self.motor_nr]:
            vel_cmd = self.speed_val[self.motor_nr]
        elif -self.speed_val[self.motor_nr] < vel_cmd < 0:
            vel_cmd = -self.speed_val[self.motor_nr]

        speed.data = np.zeros(3)
        speed.data[self.commands[self.motor_nr]] = vel_cmd
        self.publisher_1.publish(speed)
            

    def motor_stop(self):
        if self.stopped is False:
            speed = Float64MultiArray()
            speed.data = np.zeros(3)
            self.publisher_1.publish(speed)
        Extremities = Float64MultiArray()
        Extremities.data = self.ext.flatten()
        self.publisher_2.publish(Extremities)
        self.stopped = True

    def get_indeces(self, msg):
        for i, joint_name in enumerate(msg.joint_names):
            if "position" in msg.interface_values[i].interface_names:
                self.joints[i,0] = msg.interface_values[i].interface_names.index("position")
            if "velocity" in msg.interface_values[i].interface_names:
                self.joints[i,1] = msg.interface_values[i].interface_names.index("velocity")
            if "effort" in msg.interface_values[i].interface_names:   
                self.joints[i,2] = msg.interface_values[i].interface_names.index("effort")
            if "voltage" in msg.interface_values[i].interface_names:
                self.joints[i,3] = msg.interface_values[i].interface_names.index("voltage")
        self.get_logger().info(f"Indeces: {self.joints}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorCal()

    rclpy.spin(node)   

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()