# Import rclpy library so it's Node class can be used
import rclpy
from rclpy.node import Node
import numpy as np
import math

# Import built-in String type message
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class MinimalSubscriber(Node):

    # Initialize paramters
    def __init__(self):
        # super().__init__ calls the Node class’s constructor and gives it node name
        super().__init__('minimal_subscriber')
        
        # create subscriber to listen to published messages of type String on ros2 topic 'topic'
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        
        # create publisher to publish end-effector 
        self.publisher_ = self.create_publisher(String, '/topic', 10)

        # Frequency is set to 1 Hz i.e once per second 
        timer_period = 1  # 1 Hz

        # to prevent unused variable warning
        self.subscription 

    def rad_to_deg(self, rad):

        return rad * 180 /math.pi

    def listener_callback(self, msg):
        
        q1, q2, q3, q4, q5 = msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4] 

        x = math.cos(q1) * (133.4*math.cos(q2+q3+q4) + 130.2*(math.cos(q2 - 1.39)) + 124*math.cos(q2+q3))
        y = math.sin(q1) * (133.4*math.cos(q2+q3+q4) + 130.2*(math.cos(q2 - 1.39)) + 124*math.cos(q2+q3))
        z = 48163/500 - (651*math.sin(q2 - 139/100))/5 - 124*math.sin(q2 + q3) - (667*math.sin(q2 + q3 + q4))/5

        # extract x, y, z and round the values to 2 decimal places
        x, y, z = round(x,2), round(y,2), round(z,2)
        # print("X = ", x, "Y = ", y, "Z = ", z)
        # print("----------------------------------")

        self.publisher(x, y, z)

    def publisher(self, x, y, z):

        msg = String()

        x, y, z = str(x), str (y), str(z)

        msg.data = "X = " + x + ", Y = " + y + ", Z = " + z

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # oject of class MinimalSubscriber() is created
    minimal_subscriber = MinimalSubscriber()

    # loop continuously till node is terminated
    rclpy.spin(minimal_subscriber)
    
    # terminate node
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    # call main() function
    main()