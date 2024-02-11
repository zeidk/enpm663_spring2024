/**
 * @file publisher_node.cpp
 * @brief Implementation of a ROS 2 publisher node in C++.
 *
 * This file contains the implementation of the PublisherNode class, which is a
 * ROS 2 node designed to publish string messages at regular intervals. It
 * demonstrates a basic ROS 2 publisher node using rclcpp.
 */

#include <rclcpp/rclcpp.hpp>
#include "first_package/publisher_interface.hpp"

//=====================================
void PublisherNode::publish_message() {
  // Initialize the message
  std_msgs::msg::String string_msg = std_msgs::msg::String();
  string_msg.data = "Help me Obi-Wan Kenobi, you are my only hope";
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << string_msg.data);
  publisher_->publish(string_msg);
}

/**
 * @brief Main function to initialize and run the PublisherNode.
 *
 * This is the entry point of the program. It initializes the ROS 2 system,
 * creates a PublisherNode, and spins it to continuously publish messages until
 * the program is interrupted or terminated.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Execution status code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  ///< Initialize ROS 2.
  auto publisher_node = std::make_shared<PublisherNode>(
      "publisher_cpp");           ///< Create an instance of PublisherNode.
  rclcpp::spin(publisher_node);  ///< Enter a loop, pumping callbacks.
  rclcpp::shutdown();            ///< Shutdown ROS 2.
}
