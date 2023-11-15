import rclpy
from rclpy.node import Node
import numpy as np
import math

# Import built-in String type message
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from interfaces.srv import IK500

class InverseKinematicsHandler(Node):

    def __init__(self):
        super().__init__("inverse_kinematics")
        self.srv = self.create_service(IK500, "inverse_kinematics", self.inverse_kinematics_callback)
    
    def inverse_kinematics_callback(self, request, response):
        # x (position/pose) is given, q is calculated

        q_values = self.ik500(request.x, request.y, request.z, request.alpha)

        response.q1 = q_values[0]
        response.q2 = q_values[1]
        response.q3 = q_values[2]
        response.q4 = q_values[3]

        self.get_logger().info("Service returning: %s" % str(q_values))
        return response

    def ik500(self, x, y, z, alpha):
        # alpha is in radians
        
        # robot definitions
        L1 = 96.326
        L2 = math.sqrt((128**2) + (24**2))
        theta_offset = math.atan2(128, 24)
        L3 = 124
        L4 = 133.4
        
        # intermediate lengths
        r_star = math.sqrt((x**2) + (y**2)) - (math.cos(alpha) * L4)
        z_star = z - L1 - (math.sin(alpha) * L4)
        LI = math.sqrt((r_star**2) + (z_star**2))

        # intermediate angles
        T3i = math.acos(((L2**2) + (L3**2) - (LI**2)) / (2*L2*L3))
        T2A = math.asin((L3*math.sin(T3i)) / LI)
        T2B = math.atan2(z_star, r_star)
        theta_offset_2 = math.pi - theta_offset
        
        # joint variables
        T1 = math.atan2(y, x)
        T2 = theta_offset - T2B - T2A
        T3 = T3i - theta_offset_2
        T4 = alpha - T2 - T3

        # return
        return (T1, T2, T3, T4)

        
def main(args=None):
    rclpy.init(args=args)

    # oject of class MinimalSubscriber() is created
    thing = InverseKinematicsHandler()

    # loop continuously till node is terminated
    rclpy.spin(thing)
    
    # terminate node
    thing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    # call main() function
    main()