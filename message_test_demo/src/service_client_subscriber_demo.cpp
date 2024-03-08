#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "message_test_demo/service_client_subscriber_interface.hpp"

using namespace std::chrono_literals;

// aliases
using GetSpeedProfileSrv = interface_demo_msgs::srv::GetSpeedProfile;
using VehicleStatusMsg = interface_demo_msgs::msg::VehicleStatus;

//----------------------------------------------------------------
void ServiceClientSubscriberInterface::print_profile(int result) {
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
void ServiceClientSubscriberInterface::subscriber_callback(
    const VehicleStatusMsg::SharedPtr msg) {
  send_async_request(msg->speed);
  send_sync_request(msg->speed);
}

//----------------------------------------------------------------
void ServiceClientSubscriberInterface::send_async_request(double speed) {
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
  request_->speed = speed;

  auto future_result = async_client_->async_send_request(
      request_, std::bind(&ServiceClientSubscriberInterface::response_callback,
                          this, std::placeholders::_1));
}

//----------------------------------------------------------------
void ServiceClientSubscriberInterface::send_sync_request(double speed) {
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
  request_->speed = speed;

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
void ServiceClientSubscriberInterface::response_callback(
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
  auto node = std::make_shared<ServiceClientSubscriberInterface>(
      "service_client_subscriber_demo");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();  // This will start the execution
  rclcpp::shutdown();
}
