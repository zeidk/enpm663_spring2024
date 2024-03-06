#include <rclcpp/rclcpp.hpp>

#include "message_test_demo/service_server_demo_interface.hpp"
#include <interface_demo_msgs/srv/get_speed_profile.hpp>

// aliases
using GET_SPEED_PROFILE = interface_demo_msgs::srv::GetSpeedProfile;

void VehicleSpeedStatusInterface::handle_vehicle_speed_status(
    const std::shared_ptr<GET_SPEED_PROFILE::Request> request,
    std::shared_ptr<GET_SPEED_PROFILE::Response> response) {
  auto speed = request->speed;

  if (speed < 30) {
    response->status = GET_SPEED_PROFILE::Request::SLOW;
  } else if (speed >= 30 && speed < 60) {
    response->status = GET_SPEED_PROFILE::Request::NORMAL;
  } else if (speed >= 60) {
    response->status = GET_SPEED_PROFILE::Request::FAST;
  }

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Received request: " << speed << " = " << static_cast<int>(response->status));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleSpeedStatusInterface>(
      "service_server_demo");
  rclcpp::spin(node);
  rclcpp::shutdown();
}