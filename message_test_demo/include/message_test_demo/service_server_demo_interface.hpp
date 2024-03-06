#pragma once

#include <chrono>
#include <interface_demo_msgs/srv/get_speed_profile.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>


using GET_SPEED_PROFILE = interface_demo_msgs::srv::GetSpeedProfile;


class VehicleSpeedStatusInterface : public rclcpp::Node {
 public:
  VehicleSpeedStatusInterface(std::string node_name) : Node(node_name) {
    service_ = this->create_service<GET_SPEED_PROFILE>(
        "get_speed_profile",
        std::bind(&VehicleSpeedStatusInterface::handle_vehicle_speed_status,
                  this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "vehicle_speed_status server is running.");
  }

 private:
  void handle_vehicle_speed_status(
      const std::shared_ptr<GET_SPEED_PROFILE::Request> request,
      std::shared_ptr<GET_SPEED_PROFILE::Response> response);

  rclcpp::Service<GET_SPEED_PROFILE>::SharedPtr service_;
};