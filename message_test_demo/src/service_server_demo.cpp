#include <rclcpp/rclcpp.hpp>

#include "message_test_demo/service_server_demo_interface.hpp"

void VehicleSpeedStatusInterface::handle_vehicle_speed_status(
    const std::shared_ptr<GetVehicleStatus::Request> request,
    std::shared_ptr<GetVehicleStatus::Response> response) {
  auto speed = request->speed;

  if (speed < 30) {
    response->status = GetVehicleStatus::Request::SLOW;
  } else if (speed >= 30 && speed < 60) {
    response->status = GetVehicleStatus::Request::NORMAL;
  } else if (speed >= 60) {
    response->status = GetVehicleStatus::Request::FAST;
  }

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Received request: " << speed << " = " << response->status);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleSpeedStatusInterface>(
      "vehicle_speed_status_server");
  rclcpp::spin(node);
  rclcpp::shutdown();
}