/**
 * @file echo_node.cpp
 * @brief Demonstrates a simple ROS 2 node in C++ that logs a message.
 *
 * This program initializes a ROS 2 node, logs a greeting message, and then
 * spins the node to keep it alive until it is manually terminated or receives
 * a shutdown signal.
 */

#include <rclcpp/rclcpp.hpp>  // Include the necessary header to use the ROS 2 C++ client library

/**
 * @brief Main function that initializes a ROS 2 node and logs a message.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Execution status code. Returns 0 on successful execution.
 */
int main(int argc, char* argv[]) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create a shared pointer to a new node named "echo_node_cpp"
  auto node = rclcpp::Node::make_shared("echo_node_cpp");

  // Log a message including the node name using the node's logger
  RCLCPP_INFO(node->get_logger(), "Hello from %s", node->get_name());

  // Spin the node so it can perform its work (e.g., processing callbacks)
  rclcpp::spin(node);

  // Shutdown the ROS 2 system before exiting the program
  rclcpp::shutdown();

  return 0;  // Indicate successful execution
}
