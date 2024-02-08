/**
 * @file main.cpp
 * @brief Main entry point for the ROS2 node "echo_node_cpp".
 *
 * This program initializes a ROS2 node using the rclcpp library, spins the node
 * to process data callbacks, and shuts down cleanly on exit. The node
 * implemented in this example is based on the AdvancedNode class from the
 * first_package.
 *
 * @version 1.0
 * @date 2024-02-08
 */

#include "first_package/advanced_node.hpp"  // Include the header for the AdvancedNode

#include <rclcpp/rclcpp.hpp>  // Include the necessary header for ROS2 C++ client library

/**
 * @brief Main function to initialize and run the ROS2 node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Execution status code.
 */
int main(int argc, char* argv[]) {
  // Initialize ROS2 communications
  rclcpp::init(argc, argv);

  // Create a shared pointer to an instance of AdvancedNode
  auto node = std::make_shared<AdvancedNode>("echo_node_cpp");

  // Spin the node to process callbacks
  rclcpp::spin(node);

  // Shutdown ROS2 communications before exiting
  rclcpp::shutdown();

  return 0;  // Return successful execution status
}
