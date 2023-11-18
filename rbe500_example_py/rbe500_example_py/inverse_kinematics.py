import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
import math
import numpy as np

class BasicRobotControl(Node):
    def __init__(self):
        super().__init__('basic_robot_control')
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        self.send_request()

    def send_request(self):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']

        x_pose = np.array([280, 200])
        y_pose = np.array([0.0, 50])
        z_pose = np.array([128, -50])
        gamma = np.array([0.0, -90])
        counter = len(x_pose) - 1

        while (counter >= 0):

            angles = self.inverse_kinematics(x_pose[counter], y_pose[counter], z_pose[counter], gamma[counter])

            print(counter, angles)

            request.joint_position.position = [angles[0], angles[1], angles[2], angles[3], 0.05]
            request.path_time = 5.0
            self.future = self.client.call_async(request)

            counter -= 1


    def inverse_kinematics(self, xe, ye, ze, gamma):

        l12 = 130.23
        l23 = 124
        l34 = 133.4
        offset = 79.38

        g = np.radians(gamma)

        J1a, J2a, J3a = 0.0, 0.0, 0.0

        theta1 = math.atan(ye/xe)

        x3 = xe - (l34*math.cos(g))
        z3 = ze - (l34*math.sin(g))
        c = math.sqrt(x3**2 + z3**2)

        if ((l12 + l23) > c):

            a = math.acos((l12**2 + l23**2 - c**2)/(2*l12*l23))
            B = math.acos((l12**2 + c**2 - l23**2)/(2*l12*c))

            # elbow-down
            J1a = -(math.atan(z3/x3)-B - np.radians(offset))
            J2a = -(math.pi-a + np.radians(offset))
            J3a = -(g - J1a -J2a)

            # elbow-up
            J1b = -(math.atan(z3/x3)+B - np.radians(offset))
            J2b = math.pi-a -np.radians(offset)
            J3b = -(g - J1b - J2b)

        else:
            print("dimension error!")

        angles = np.array([theta1, J1b, J2b, math.pi - J3b])

        return angles


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
