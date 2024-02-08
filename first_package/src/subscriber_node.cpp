/**
 * @file subscriber_node_main.cpp
 * @brief Example of a ROS 2 Subscriber Node in C++.
 *
 * This file demonstrates the creation and usage of a ROS 2 subscriber node
 * using the rclcpp library. The SubscriberNode class is utilized here to
 * subscribe to String messages on a topic.
 */

#include "subscriber_node.hpp"  // Custom subscriber node header

#include <std_msgs/msg/string.hpp>  // Include for ROS 2 string messages
#include <string>                   // Include for std::string

using namespace std::chrono_literals;  // Allows using the 'ms' literal for
                                       // milliseconds

//=====================================
void SubscriberNode::help_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Received data: " << msg->data);
}

/**
 * @brief Main function to initialize and run the ROS 2 subscriber node.
 *
 * Initializes the ROS 2 system, creates a SubscriberNode, and keeps it spinning
 * to listen for incoming messages. Shuts down the ROS 2 system before exiting.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Execution status code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize ROS 2
  auto subscriber_node = std::make_shared<SubscriberNode>(
      "help_sub_cpp");  // Create an instance of SubscriberNode
  rclcpp::spin(
      subscriber_node);  // Keep the node alive and listening for messages
  rclcpp::shutdown();    // Shut down ROS 2
  return 0;              // Exit the program
}
