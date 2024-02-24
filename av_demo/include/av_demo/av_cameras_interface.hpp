/**
 * @file av_sensors_interface.hpp
 * @author Zeid Kootbally (zeidk@umd.edu)
 * @brief File containing the AVSensorsInterface class definition
 * @version 0.1
 * @date 2024-02-21
 *
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// aliases
using camera_msg = sensor_msgs::msg::Image;

class AVCamerasInterface : public rclcpp::Node {
 public:
  AVCamerasInterface(std::string node_name) : Node(node_name) {
    // initialize the message
    camera_msg_ = sensor_msgs::msg::Image();

    // publisher
    camera_publisher_ = this->create_publisher<camera_msg>("av_camera", 10);

    // timers for publishing
    camera_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)((1 / 15) * 1000)),
        std::bind(&AVCamerasInterface::camera_timer_callback, this));
  }

 private:
  // message objects
  camera_msg camera_msg_;  //!< Message object for camera
  // timer objects
  rclcpp::TimerBase::SharedPtr camera_timer_;  //!< Timer object for camera
  // publisher objects
  rclcpp::Publisher<camera_msg>::SharedPtr
      camera_publisher_;  //!< Publisher object for camera

  // callbacks
  /**
   * @brief  Timer callback for camera
   *
   */
  void camera_timer_callback();
};