#pragma once

#include <radar_msgs/msg/radar_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// aliases
using camera_msg = sensor_msgs::msg::Image;
using lidar_msg = sensor_msgs::msg::LaserScan;
using radar_msg = radar_msgs::msg::RadarScan;

class AVActionsInterface : public rclcpp::Node {
 public:
  AVActionsInterface(std::string node_name) : Node(node_name) {
    // initialize the message
    camera_msg_ = sensor_msgs::msg::Image();
    lidar_msg_ = sensor_msgs::msg::LaserScan();
    radar_msg_ = radar_msgs::msg::RadarScan();

    // subscribers
    // Since we are not doing much with the received data, we are using lambdas
    // instead of member functions for the callbacks. If you want to do more
    // complex things with the received messages, use member functions with std::bind
    camera_subscriber_ = this->create_subscription<camera_msg>(
        "av_camera", 10, [this](const camera_msg::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Camera data received");
          camera_msg_ = *msg;
        });

    lidar_subscriber_ = this->create_subscription<lidar_msg>(
        "av_lidar", 10, [this](const lidar_msg::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Lidar data received");
          lidar_msg_ = *msg;
        });

    radar_subscriber_ = this->create_subscription<radar_msg>(
        "av_radar", 10, [this](const radar_msg::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Radar data received");
          radar_msg_ = *msg;
        });
 
    // timers for publishing
    data_fusion_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)((1 / 20) * 1000)),
        std::bind(&AVActionsInterface::data_fusion_callback, this));
  }

 private:
  // message objects
  camera_msg camera_msg_;  //!< Message object for camera
  lidar_msg lidar_msg_;    //!< Message object for lidar
  radar_msg radar_msg_;    //!< Message object for radar
  // timer objects
  rclcpp::TimerBase::SharedPtr data_fusion_timer_;  //!< Timer object for data fusion
  // subscriber objects
  rclcpp::Subscription<camera_msg>::SharedPtr
      camera_subscriber_;  //!< Subscriber object for camera
  rclcpp::Subscription<lidar_msg>::SharedPtr
      lidar_subscriber_;  //!< Subscriber object for lidar
  rclcpp::Subscription<radar_msg>::SharedPtr
      radar_subscriber_;  //!< Subscriber object for radar

  // callbacks

  /**
   * @brief  Timer callback for data fusion
   * 
   * This callback uses lidar and radar data to create a fused data
   *
   */
  void data_fusion_callback();
};