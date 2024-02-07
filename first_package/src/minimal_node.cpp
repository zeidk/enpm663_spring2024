// ROS 2 C++ library for using ROS 2 functionalities.
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  // Initialize the ROS 2 communication system.
  rclcpp::init(argc, argv);
  // Create a shared pointer to a new node named "minimal_node_cpp".
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("echo_node_cpp");
  // Log a "Hello, world!" message.
  RCLCPP_INFO(node->get_logger(), "Hello from %s", node->get_name());
  // Shutdown the ROS 2 communication system before exiting the program.
  // This is a clean-up step to ensure all resources are properly released.
  // spin the node
  rclcpp::spin(node);
  rclcpp::shutdown();
}