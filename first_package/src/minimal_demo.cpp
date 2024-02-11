/**
 * @file minimal_demo.cpp
 * @brief Demonstrates a simple ROS 2 node in C++ that logs a message.
 *
 * This program initializes a ROS 2 node and logs a greeting message.
 */

// Include the necessary header to use the ROS 2 C++ client library
#include <rclcpp/rclcpp.hpp>  

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

  // Create a shared pointer to a new node named "minimal_cpp"
  auto node = rclcpp::Node::make_shared("minimal_cpp");

  // Log a message including the node name using the node's logger
  RCLCPP_INFO(node->get_logger(), "Hello from %s", node->get_name());

  // Shutdown the ROS 2 system before exiting the program
  rclcpp::shutdown();
}
