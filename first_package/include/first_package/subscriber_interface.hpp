/**
 * @file subscriber_interface.hpp
 * @brief Declares the SubscriberNode class for subscribing to "leia" topic
 * messages.
 *
 * This file contains the declaration of the SubscriberNode class, which
 * extends the rclcpp::Node class to create a ROS2 subscriber node that listens
 * to messages of type std_msgs::msg::String published on the "leia" topic.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class SubscriberNode
 * @brief A ROS2 subscriber node class.
 *
 * SubscriberNode is a class derived from rclcpp::Node designed to subscribe to
 * string messages on a topic named "leia". It demonstrates the basic structure
 * of a ROS2 subscriber in C++.
 */
class SubscriberNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs a SubscriberNode with a specified name.
   *
   * Initializes the subscriber to listen on the "leia" topic for messages
   * of type std_msgs::msg::String. It binds the receive_message function to
   * be called upon receiving a message.
   *
   * @param node_name The name of the node.
   */
  SubscriberNode(std::string node_name) : Node(node_name) {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "leia", 10,
        std::bind(&SubscriberNode::receive_message, this, std::placeholders::_1));
  }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;  ///< Shared pointer to the subscription object.
  
  /**
   * @brief Callback function for subscription to "leia" topic.
   *
   * This function is called whenever a new message is published on the "leia"
   * topic. It processes the received message.
   *
   * @param msg The message received from the topic.
   */
  void receive_message(const std_msgs::msg::String::SharedPtr msg);
};
