// ROS 2 C++ library for using ROS 2 functionalities.
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  // 1. Initialize the ROS 2 communication system.
  rclcpp::init(argc, argv);
  // 2. Create a shared pointer to a new node named
  auto node = rclcpp::Node::make_shared("echo_node_cpp");
  RCLCPP_INFO(node->get_logger(), "Hello from %s", node->get_name());
  // Log a "Hello, world!" message.RCLCPP_INFO(
  rclcpp::spin(node);
  // 3. Shutdown the ROS 2 communication system before exiting the program.
  rclcpp::shutdown();
}