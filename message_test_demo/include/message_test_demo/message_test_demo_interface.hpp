#pragma once

#include <interface_demo_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

// timer
class MessageTestDemoInterface : public rclcpp::Node {
 public:
  MessageTestDemoInterface(std::string node_name) : Node(node_name) {
    // initialize the message
    msg_ = interface_demo_msgs::msg::VehicleStatus();

    // initialize the publisher
    publisher_ =
        this->create_publisher<interface_demo_msgs::msg::VehicleStatus>(
            "vehicle_status", 10);
    // initialize the timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(4),
        std::bind(&MessageTestDemoInterface::timer_callback, this));
  }

 private:
  // message object
  interface_demo_msgs::msg::VehicleStatus msg_;
  // timer object
  rclcpp::TimerBase::SharedPtr timer_;
  // publisher object
  rclcpp::Publisher<interface_demo_msgs::msg::VehicleStatus>::SharedPtr
      publisher_;

  // methods
  void timer_callback();
};