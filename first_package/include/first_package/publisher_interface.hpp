#pragma once

/**
 * @file publisher_interface.hpp
 * @brief Declares the PublisherNode class for publishing string messages at
 * regular intervals.
 *
 * This file defines the PublisherNode class, which inherits from rclcpp::Node.
 * It demonstrates creating a simple ROS2 publisher in C++ that periodically
 * publishes messages to a topic.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class PublisherNode
 * @brief A class that publishes string messages at fixed intervals.
 *
 * PublisherNode is a ROS2 node that periodically publishes string messages.
 */
class PublisherNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs a PublisherNode with a given name.
   *
   * @param node_name The name of the node.
   * Initializes the publisher, timer, and sets up the timer callback for
   * publishing messages.
   */
  PublisherNode(std::string node_name) : Node(node_name) {

    // Setup a timer to call timer_callback every 1000 milliseconds
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000.0)),
        std::bind(&PublisherNode::publish_message, this));

    // Initialize the publisher on topic "help" with a queue size of 10
    publisher_ = this->create_publisher<std_msgs::msg::String>("leia", 10);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer to trigger publishing.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      publisher_;  ///< The publisher object.

  /**
   * @brief Timer callback function that publishes a message.
   *
   * This function constructs a message and publishes the message.
   */
  void publish_message();
};
