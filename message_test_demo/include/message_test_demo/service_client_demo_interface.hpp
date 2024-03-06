#pragma once

#include <rclcpp/rclcpp.hpp>
#include <interface_demo_msgs/srv/get_speed_profile.hpp>
#include <interface_demo_msgs/msg/vehicle_status.hpp>
#include <chrono>
#include <iostream>
#include <memory>


// aliases
using GET_SPEED_PROFILE = interface_demo_msgs::srv::GetSpeedProfile;
using VEHICLE_STATUS = interface_demo_msgs::msg::VehicleStatus;
/**
 * @brief Class for the client
 *
 */
class GetVehicleStatusClientInterface : public rclcpp::Node
{
public:
    GetVehicleStatusClientInterface(std::string node_name) : Node(node_name), current_speed_(-1.0)
    {
        // initialize the subscriber object
        // "vehicle_status" is the name of the topic
        subscriber_ = this->create_subscription<VEHICLE_STATUS>("vehicle_status", 10,
                                                                     std::bind(&GetVehicleStatusClientInterface::subscriber_callback, this, std::placeholders::_1));
        // initialize the client object
        // "get_vehicle_status" is the name of the service
        client_ = this->create_client<GET_SPEED_PROFILE>("get_speed_profile");

        // timer callback function to send requests to the server at regular intervals (2 seconds)
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)),
                                         std::bind(&GetVehicleStatusClientInterface::timer_callback, this));
    }

private:
    // current speed received by the subscriber callback function
    double current_speed_;
    // timer object to call the client at regular intervals
    rclcpp::TimerBase::SharedPtr timer_;
    // client object
    rclcpp::Client<GET_SPEED_PROFILE>::SharedPtr client_;
    // subscriber object to interface_demo_msgs::msg::VehicleStatus
    rclcpp::Subscription<VEHICLE_STATUS>::SharedPtr subscriber_;
    // subscriber callback function
    void subscriber_callback(const VEHICLE_STATUS::SharedPtr msg);

    // send request to the server from a timer
    void send_request_from_timer();
    // send request to the server from a subscriber callback
    void send_request_from_subscriber(double speed);

    // timer callback function to send requests to the server at regular intervals
    void timer_callback();
    /**
     * @brief Callback function for the client
     *
     * This function is called when the client receives a response from the server
     * @param future Shared pointer to the future object.
     *  A future is a value that indicates whether the call and response is finished (not the value of the response itself)
     */
    void response_callback(rclcpp::Client<GET_SPEED_PROFILE>::SharedFuture future);
};