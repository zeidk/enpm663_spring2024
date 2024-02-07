#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// timer
class PublisherNode : public rclcpp::Node {
 public:
  PublisherNode(std::string node_name) : Node(node_name), counter_{0} {
    // initialize the message
    msg_ = std_msgs::msg::String();
  
    // timer callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(500.0)),
        std::bind(&PublisherNode::timer_callback, this));

    // publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("help", 10);
  }

 private:
  // attributes
  std_msgs::msg::String msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  unsigned counter_;

  // methods
  void timer_callback();
};