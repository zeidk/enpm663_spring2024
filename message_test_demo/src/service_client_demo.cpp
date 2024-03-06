#include <rclcpp/rclcpp.hpp>
#include "message_test_demo/service_client_demo_interface.hpp"
#include <chrono>
#include <memory>
#include <iostream>
#include <string>

using namespace std::chrono_literals;

// aliases
using GET_SPEED_PROFILE = interface_demo_msgs::srv::GetSpeedProfile;
using VEHICLE_STATUS = interface_demo_msgs::msg::VehicleStatus;

//----------------------------------------------------------------
void GetVehicleStatusClientInterface::subscriber_callback(const VEHICLE_STATUS::SharedPtr msg)
{
    // store the speed to be used in the timer callback function
    // current_speed_ = msg->speed;
    RCLCPP_INFO(this->get_logger(), "Sending request from subscriber callback: %f", msg->speed);
    send_request_from_subscriber(msg->speed);
}

//----------------------------------------------------------------
void GetVehicleStatusClientInterface::timer_callback()
{
    if (current_speed_ > 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Sending request from timer callback: %f", current_speed_);
        send_request_from_timer();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for vehicle status...");
    }
}

//----------------------------------------------------------------
void GetVehicleStatusClientInterface::response_callback(rclcpp::Client<GET_SPEED_PROFILE>::SharedFuture future)
{
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready)
    {
        auto result = static_cast<int>(future.get()->status);

        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Vehicle Status: SLOW");
        }
        else if (result == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Vehicle Status: NORMAL");
        }
        else if (result == 2)
        {
            RCLCPP_INFO(this->get_logger(), "Vehicle Status: FAST");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Vehicle Status: UNKNOWN");
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

//----------------------------------------------------------------
void GetVehicleStatusClientInterface::send_request_from_timer()
{
    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Create a request and send it to the server
    auto request = std::make_shared<GET_SPEED_PROFILE::Request>();
    request->speed = current_speed_;

    auto future_result = client_->async_send_request(request, std::bind(&GetVehicleStatusClientInterface::response_callback, this, std::placeholders::_1));
}

//----------------------------------------------------------------
void GetVehicleStatusClientInterface::send_request_from_subscriber(double speed)
{
    // deactivate the timer is we are calling this function
    timer_->cancel();
    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Create a request and send it to the server
    auto request = std::make_shared<GET_SPEED_PROFILE::Request>();
    request->speed = speed;

    auto future_result = client_->async_send_request(request, std::bind(&GetVehicleStatusClientInterface::response_callback, this, std::placeholders::_1));
}

//----------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GetVehicleStatusClientInterface>("service_client_demo");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}