import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np
import math
from std_msgs.msg import Float32
from dynamixel_sdk_custom_interfaces.msg import SetCurrent
from dynamixel_sdk_custom_interfaces.srv import GetCurrent
from dynamixel_sdk_custom_interfaces.srv import GetPosition

class PD_Controller(Node):
   
    def __init__(self):
        self.target_position = None
        self.last_position = None
        self.Kp = 1.0
        self.Kd = 0.0
        super().__init__("pd_controller")
        # self.target_subscriber = self.create_subscription(GetPosition, "/target_pos", self.update_target_callback, 10)
        self.position_client = self.create_client(GetPosition, "/get_position") #TODO: double check topic name
        self.effort_publisher = self.create_publisher(SetCurrent, "/set_current", 10)
       
    def update_target_callback(self, msg):
        # we can ignore the target ID, we only care about the position value (we know ID must be motor 14)
        self.target_position = msg.position
        self.get_logger().info("Setting Target Position to: " + str(self.target_position))
   
    def pd_update(self):
        pd_update_request = GetPosition.Request()
        pd_update_request.id = 14
        position_future.add_done_callback(self.q4_callback)
        position_future = self.position_client.call_async(pd_update_request)
        rclpy.spin_until_future_complete(self, position_future)
        # done, everything else is in self.q4_callback
       
    def q4_callback(self, position_future):
        result = position_future.result()
        curr_pos = result.position # current position of motor 4
        self.get_logger().info("Current position: " + str(curr_pos))

        self.p_component = 0 # initial values will be overwritten
        self.d_component = 0

        if self.target_position is not None:
            # error is target - current position
            p_component = self.Kp * (self.target_position - curr_pos)
        if self.last_position is not None:
            # derivative doesn't care about the target position, only the last position
            d_component = self.Kd * (curr_pos - self.last_position)
            #TODO: take time into account
        # self.get_clock().now() # does this actually work?

        self.last_position = curr_pos # save last position for derivative component
       
        control_value = p_component + d_component

        msg = SetCurrent()
        msg.data.id = 14
        msg.data.current = control_value
        self.effort_publisher.publish(msg) # send motor current to relevant topic
       
def main(args=None):    
    rclpy.init(args=args)

    controller = PD_Controller()

    while rclpy.ok():
        controller.get_logger().info("Starting loop")
        rclpy.spin_once(controller)
        if controller.future.done():
            try:
                response = controller.future.result()
            except Exception as e:
                controller.get_logger().error('Service call failed %r' % (e,))
            break
        controller.pd_update() # do AFAP
       
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


