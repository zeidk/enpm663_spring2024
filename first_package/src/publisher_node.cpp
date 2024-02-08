/**
 * @file publisher_node.cpp
 * @brief Implementation of a ROS 2 publisher node in C++.
 *
 * This file contains the implementation of the PublisherNode class, which is a
 * ROS 2 node designed to publish string messages at regular intervals. It
 * demonstrates a basic ROS 2 publisher node using rclcpp.
 */

#include "first_package/publisher_node.hpp"

#include <rclcpp/rclcpp.hpp>

//=====================================
void PublisherNode::timer_callback() {
  msg_.data = std::to_string(counter_) +
              ": Help me Obi-Wan Kenobi, you are my only hope.";  ///< Assign
                                                                  ///< message
                                                                  ///< data.
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Publishing: " << msg_.data);  ///< Log the message being published.
  publisher_->publish(msg_);         ///< Publish the message.
  counter_++;                        ///< Increment the counter.
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
      "help_pub_cpp");           ///< Create an instance of PublisherNode.
  rclcpp::spin(publisher_node);  ///< Enter a loop, pumping callbacks.
  rclcpp::shutdown();            ///< Shutdown ROS 2.
}
