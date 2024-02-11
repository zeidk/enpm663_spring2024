/**
 * @file advanced_interface.hpp
 * @brief Defines the AdvancedNode class as a subclass of rclcpp::Node.
 *
 * This header file provides the declaration of the AdvancedNode class, which
 * extends the functionality of rclcpp::Node to demonstrate custom node creation
 * in ROS 2. It logs a greeting message upon initialization.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>  // Include the ROS 2 C++ client library support.

/**
 * @class AdvancedNode
 * @brief A simple ROS 2 node class that extends rclcpp::Node.
 *
 * AdvancedNode is a demonstration class that inherits from rclcpp::Node. It
 * provides an example of how to define a custom node in ROS 2. Upon creation,
 * it logs a greeting message that includes the node's name.
 */
class AdvancedNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new AdvancedNode object.
   *
   * Initializes a new instance of the AdvancedNode, setting up the node's name
   * and logging a greeting message.
   *
   * @param node_name The name of the node, passed to the rclcpp::Node
   * constructor.
   */
  AdvancedNode(std::string node_name) : Node(node_name) {
    RCLCPP_INFO(this->get_logger(), "Hello from %s", this->get_name());
    // Logs a greeting message using the ROS 2 logging facility.
  }
};
