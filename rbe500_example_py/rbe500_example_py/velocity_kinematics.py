import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
import math
import numpy as np
import time

class VelocityKinematicsNode(Node):
    def __init__(self):
        super().__init__('velocity_kinematics')
        
        # arm link lengths
        self.L12 = 96.326
        self.L3 = 130.23
        self.L4 = 124
        self.L5 = 133.4
        
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.gripper_client = self.create_client(SetJointPosition, '/goal_tool_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        self.send_request()

    def makeJacobian(self, q1, q2, q3, q4):

        # assume joint positions are passed in as radians
        
        q2preload = math.radians(10.807)
        q34preload = math.radians(90)
        
        x_star = (self.L3 * math.sin(q2 + q2preload)) + (L4 * math.sin(q2+q3+q34preload)) + (L5 * math.sin(q2+q3+q4+q34preload))
        
        q4xy = L5 * math.cos(q2+q3+q4+q34preload)
        q3xy = q4xy + (L4 * math.cos(q2+q3+q34preload))
        q2xy = q3xy + (L3 * math.cos(q2+q2preload))
        
        q4z = -(L5 * math.sin(q2+q3+q4+q34preload))
        q3z = q4z - (L4*math.sin(q2+q3+q34preload))
        q2z = q3z - (L3*math.sin(q2+q2preload))
        
        c1 = math.cos(q1)
        s1 = math.sin(q1)
        
        #                       q1dot       q2dot       q3dot       q4dot
        jacob = np.array([[-x_star * s1,  c1 * q2xy,  c1 * q3xy,  c1 * q4xy],   # x
                          [ x_star * c1,  s1 * q2xy,  s1 * q3xy,  s1 * q4xy],   # y
                          [           0,        q2z,        q3z,        q4z]])  # z

        return jacob


def main(args=None):
    rclpy.init(args=args)

    basic_robot_control = BasicRobotControl()

    while rclpy.ok():
        rclpy.spin_once(basic_robot_control)
        if basic_robot_control.future.done():
            try:
                response = basic_robot_control.future.result()
            except Exception as e:
                basic_robot_control.get_logger().error('Service call failed %r' % (e,))
            break

    basic_robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
