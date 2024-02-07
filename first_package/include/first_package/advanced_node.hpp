#pragma once

#include <rclcpp/rclcpp.hpp>

class AdvancedNode : public rclcpp::Node {
 public:
  AdvancedNode(std::string node_name) : Node(node_name) {
    RCLCPP_INFO(this->get_logger(), "Hello from %s", this->get_name());
  }

 private:
};