#pragma once

#include <chrono>
#include <interface_demo_msgs/srv/get_vehicle_status.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

// aliases
using GetVehicleStatus = interface_demo_msgs::srv::GetVehicleStatus;
class VehicleSpeedStatusInterface : public rclcpp::Node {
 public:
  VehicleSpeedStatusInterface(std::string node_name) : Node(node_name) {
    service_ = this->create_service<GetVehicleStatus>(
        "vehicle_speed_status",
        std::bind(&VehicleSpeedStatusInterface::handle_vehicle_speed_status,
                  this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "vehicle_speed_status server is running.");
  }

 private:
  void handle_vehicle_speed_status(
      const std::shared_ptr<GetVehicleStatus::Request> request,
      std::shared_ptr<GetVehicleStatus::Response> response);

  rclcpp::Service<GetVehicleStatus>::SharedPtr service_;
};