#pragma once

#include <chrono>
#include <interface_demo_msgs/srv/get_speed_profile.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>


using GetSpeedProfileSrv = interface_demo_msgs::srv::GetSpeedProfile;


class VehicleSpeedStatusInterface : public rclcpp::Node {
 public:
  VehicleSpeedStatusInterface(std::string node_name) : Node(node_name) {
    service_ = this->create_service<GetSpeedProfileSrv>(
        "get_speed_profile",
        std::bind(&VehicleSpeedStatusInterface::handle_request,
                  this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "vehicle_speed_status server is running.");
  }

 private:
  void handle_request(
      const std::shared_ptr<GetSpeedProfileSrv::Request> request,
      std::shared_ptr<GetSpeedProfileSrv::Response> response);

  // Service object
  rclcpp::Service<GetSpeedProfileSrv>::SharedPtr service_;
};