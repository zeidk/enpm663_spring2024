/**
 * @file av_sensors_interface.hpp
 * @author Zeid Kootbally (zeidk@umd.edu)
 * @brief File containing the AVSensorsInterface class definition
 * @version 0.1
 * @date 2024-02-21
 *
 */

#pragma once

#include <radar_msgs/msg/radar_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// aliases
using camera_msg = sensor_msgs::msg::Image;
using lidar_msg = sensor_msgs::msg::LaserScan;
using radar_msg = radar_msgs::msg::RadarScan;

// timer
class AVSensorsInterface : public rclcpp::Node {
 public:
  AVSensorsInterface(std::string node_name) : Node(node_name) {
    // initialize the message
    camera_msg_ = sensor_msgs::msg::Image();
    lidar_msg_ = sensor_msgs::msg::LaserScan();
    radar_msg_ = radar_msgs::msg::RadarScan();

    // publisher
    camera_publisher_ = this->create_publisher<camera_msg>("av_camera", 10);
    lidar_publisher_ = this->create_publisher<lidar_msg>("av_lidar", 10);
    radar_publisher_ = this->create_publisher<radar_msg>("av_radar", 10);

    // parameters
    // this->declare_parameter("frequency", 1.0);
    // this->declare_parameter("bandwidth", 2.5);
    // this->declare_parameter("name", "camera");
    // camera_frequency_ = this->get_parameter("frequency").as_double();
    // camera_bandwidth_ = this->get_parameter("bandwidth").as_double();
    // camera_name_ = this->get_parameter("name").as_string();

    // publisher timers
    camera_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)((1 / 10) * 1000)),
        std::bind(&AVSensorsInterface::camera_timer_callback, this));
    lidar_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)((1 / 10) * 1000)),
        std::bind(&AVSensorsInterface::lidar_timer_callback, this));
    radar_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)((1 / 10) * 1000)),
        std::bind(&AVSensorsInterface::radar_timer_callback, this));
  }

 private:
  // message objects
  camera_msg camera_msg_;  //!< Message object for camera
  lidar_msg lidar_msg_;    //!< Message object for lidar
  radar_msg radar_msg_;    //!< Message object for radar
  // timer objects
  rclcpp::TimerBase::SharedPtr camera_timer_;  //!< Timer object for camera
  rclcpp::TimerBase::SharedPtr lidar_timer_;   //!< Timer object for lidar
  rclcpp::TimerBase::SharedPtr radar_timer_;   //!< Timer object for radar
  // publisher objects
  rclcpp::Publisher<camera_msg>::SharedPtr
      camera_publisher_;  //!< Publisher object for camera
  rclcpp::Publisher<lidar_msg>::SharedPtr
      lidar_publisher_;  //!< Publisher object for lidar
  rclcpp::Publisher<radar_msg>::SharedPtr
      radar_publisher_;  //!< Publisher object for radar

  // callbacks

  /**
   * @brief  Timer callback for camera
   *
   */
  void camera_timer_callback();
    /**
     * @brief  Timer callback for lidar
     *
     */
  void lidar_timer_callback();
    /**
     * @brief  Timer callback for radar
     *
     */
  void radar_timer_callback();
};