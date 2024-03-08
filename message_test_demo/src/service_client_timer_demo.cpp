#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "message_test_demo/service_client_timer_interface.hpp"

using namespace std::chrono_literals;

// aliases
using GetSpeedProfileSrv = interface_demo_msgs::srv::GetSpeedProfile;
using VehicleStatusMsg = interface_demo_msgs::msg::VehicleStatus;

//----------------------------------------------------------------
void ServiceClientTimerInterface::timer_callback() {
  if (current_speed_ > 0.0) {
    // send requests to the server
    send_async_request();
    send_sync_request();
  } else {
    RCLCPP_INFO(this->get_logger(), "Waiting for vehicle status...");
  }
}

//----------------------------------------------------------------
void ServiceClientTimerInterface::print_profile(int result) {
  if (result == 0) {
    RCLCPP_INFO(get_logger(), "Vehicle Status: SLOW");
  } else if (result == 1) {
    RCLCPP_INFO(get_logger(), "Vehicle Status: NORMAL");
  } else if (result == 2) {
    RCLCPP_INFO(get_logger(), "Vehicle Status: FAST");
  } else {
    RCLCPP_INFO(get_logger(), "Vehicle Status: UNKNOWN");
  }
}

//----------------------------------------------------------------
void ServiceClientTimerInterface::subscriber_callback(
    const VehicleStatusMsg::SharedPtr msg) {
  current_speed_ = msg->speed;
}

//----------------------------------------------------------------
void ServiceClientTimerInterface::send_async_request() {
  // Wait for the service to become available
  while (!async_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
  // Create a request and send it to the server
  // auto request = std::make_shared<GetSpeedProfileSrv::Request>();
  request_->speed = current_speed_;

  auto future_result = async_client_->async_send_request(
      request_, std::bind(&ServiceClientTimerInterface::response_callback, this,
                          std::placeholders::_1));
}

//----------------------------------------------------------------
void ServiceClientTimerInterface::send_sync_request() {
  // Wait for the service to become available
  while (!sync_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
  // Create a request and send it to the server
  // auto request = std::make_shared<GetSpeedProfileSrv::Request>();
  request_->speed = current_speed_;

  // send the request to the server and wait for the response
  // this is a synchronous call
  auto result_future = sync_client_->async_send_request(request_);

  try
{
    auto response = result_future.get();
    if (response->status)
    {
        print_profile(response->status);
    }
}
catch (const std::exception &e)
{
    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
}
}

//----------------------------------------------------------------
void ServiceClientTimerInterface::response_callback(
    rclcpp::Client<GetSpeedProfileSrv>::SharedFuture future) {
  auto status = future.wait_for(1s);
  if (status == std::future_status::ready) {
    auto result = static_cast<int>(future.get()->status);
    print_profile(result);
  }
}

//----------------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceClientTimerInterface>(
      "service_client_timer_demo");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();  // This will start the execution
  rclcpp::shutdown();
}
