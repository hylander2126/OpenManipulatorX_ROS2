import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
import math
import numpy as np
import time

class BasicRobotControl(Node):
    def __init__(self):
        super().__init__('basic_robot_control')
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.gripper_client = self.create_client(SetJointPosition, '/goal_tool_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        self.send_request()

    def send_request(self):
        request = SetJointPosition.Request()
        gripper_request = SetJointPosition.Request()
        request.planning_group = ''
        gripper_request.planning_group = 'gripper'
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        gripper_request.joint_position.joint_name = ['gripper']

        gripper_request.joint_position.max_accelerations_scaling_factor = 1.0
        gripper_request.joint_position.max_velocity_scaling_factor = 1.0
        gripper_request.path_time = 2.0

        # x_pose = np.array([140, 140, 140, 140, 281])
        # y_pose = np.array([190, 190, -180, -180, 0.0])
        # z_pose = np.array([-50, 50, -50, 50.0, 128])
        # gamma = np.array([-90, -90, -90, -90, 0.0])

        time.sleep(4)

        x_pose = np.array([160, 160, 160, 160, 160, 160, 280])
        y_pose = np.array([-100.0, -100.0, -100.0, 100, 100, 100, 0.0])
        z_pose = np.array([0.0, -60, 0.0, 0.0, -60, 0.0, 128])
        gamma = np.array([-90, -90, -90, -90, -90, -90, 0.0])

        gripper_open = 0.01
        gripper_close = -0.01
        gripper = np.array([gripper_open, gripper_close, gripper_close, gripper_close, 
                            gripper_open, gripper_close, gripper_close])

        counter = 0

        while (counter < len(x_pose)):

            angles = self.inverse_kinematics(x_pose[counter], y_pose[counter], z_pose[counter], gamma[counter])

            # print(counter, angles*180/math.pi)

            print("Reached destination ", counter)

            request.joint_position.position = [angles[0], angles[1], angles[2], angles[3], 0.05]
            request.path_time = 5.0
            self.future = self.client.call_async(request)

            time.sleep(6)

            gripper_request.joint_position.position = [gripper[counter]]
            self.future = self.gripper_client.call_async(gripper_request)

            time.sleep(0.5)

            counter += 1


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

            J1b_new = math.atan(z3/x3) + B
            J2b_new = -(math.pi - a)

            J3b_new = -(g - J1b_new - J2b_new)

        else:
            print("dimension error!")

        angles = np.array([theta1, J1b, J2b, J3b_new])

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
