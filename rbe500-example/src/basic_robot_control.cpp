#include "rclcpp/rclcpp.hpp"
#include "open_manipulator_msgs/srv/set_joint_position.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
/*
void callback(const rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture future){}
*/


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);


  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("basic_robot_control");
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr client =
    node->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path");

  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->planning_group = "";
  request->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4", "gripper"};
  request->joint_position.position = {1, 0, 0, 0, 0};
  request->path_time = 5.0;
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");   
      return 0;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  }
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
