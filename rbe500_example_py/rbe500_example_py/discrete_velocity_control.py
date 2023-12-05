import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import GetJointVelocity
import sys
import time

class DiscreteVelocityController(Node):
    def __init__(self):
        super().__init__('discrete_velocity_control')
        self.position_reference = [-1.0, 0.0, 0.0, 0.0, 0.0] # joint space, not including gripper
        self.set_velocity = [0.0, 30.0, 0.0] # mm/s
        self.assumed_velocity_discrete_time_ms = 100.0
        
        self.arm_client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the arm service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Arm service not available, waiting again...')
            
        self.vel_client = self.create_client(GetJointVelocity, 'inverse_vk')
        while not self.vel_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the VK service. Exiting.")
                sys.exit(0)
            self.get_logger().info("VK service not available, waiting again...")
        self.send_position() # i.e. go to the home position over 5 seconds (NOTE! the array here is a delta from the above definition of position_reference)
    
    def send_position(self):
        self.get_logger().info("Sending position: " + str(self.position_reference))
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        
        request.joint_position.position = self.position_reference
        request.path_time = self.assumed_velocity_discrete_time_ms / 1000.0

        self.future = self.arm_client.call_async(request)
    
    def tick_velocity(self):
        # assuming linearVelocity is a list of [x, y, z] velocities in mm/s
        # # assumed 100Hz refresh rate; this value needs tuning
        request = GetJointVelocity.Request()
        # first we set the current position of the arm
        # then we set the target velocity
        request.q1 = self.position_reference[0]
        request.q2 = self.position_reference[1]
        request.q3 = self.position_reference[2]
        request.q4 = self.position_reference[3]
        
        request.xd = self.set_velocity[0]
        request.yd = self.set_velocity[1]
        request.zd = self.set_velocity[2]
        
        joints_future = self.vel_client.call_async(request)
        
        joints_future.add_done_callback(self.joints_future_callback)
    
    def joints_future_callback(self, joints_future):    
        self.get_logger().info("Recieved velocities.")
        joint_vels = joints_future.result()
        self.position_reference[0] += joint_vels.qd1 * self.assumed_velocity_discrete_time_ms / 1000.0
        self.position_reference[1] += joint_vels.qd2 * self.assumed_velocity_discrete_time_ms / 1000.0
        self.position_reference[2] += joint_vels.qd3 * self.assumed_velocity_discrete_time_ms / 1000.0
        self.position_reference[3] += joint_vels.qd4 * self.assumed_velocity_discrete_time_ms / 1000.0
    
def main(args=None):    
    rclpy.init(args=args)

    controller = DiscreteVelocityController()

    while rclpy.ok():
        controller.get_logger().info("Starting loop")
        rclpy.spin_once(controller,timeout_sec=(10/1000.0))
        if controller.future.done():
            try:
                response = controller.future.result()
            except Exception as e:
                controller.get_logger().error('Service call failed %r' % (e,))
            break
        controller.tick_velocity() # do these every ~10ms
        controller.send_position()
        time.sleep(100/1000.0) # delay for 10ms before continuing
        
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
