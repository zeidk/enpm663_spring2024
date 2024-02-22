#include <radar_msgs/msg/radar_return.hpp>
#include <rclcpp/rclcpp.hpp>

#include "av_demo/av_sensors_interface.hpp"

void AVSensorsInterface::camera_timer_callback() {
  // fill the message
  camera_msg_.header.stamp = this->now();
  camera_msg_.header.frame_id = "av_camera";
  camera_msg_.height = 480;
  camera_msg_.width = 640;
  camera_msg_.encoding = "rgb8";
  camera_msg_.is_bigendian = false;
  camera_msg_.step = 640 * 3;
  camera_msg_.data.resize(640 * 480 * 3);
  for (size_t i = 0; i < camera_msg_.data.size(); i += 3) {
    camera_msg_.data[i] = 255;
    camera_msg_.data[i + 1] = 0;
    camera_msg_.data[i + 2] = 0;
  }
  // publish the message
  camera_publisher_->publish(camera_msg_);
}

void AVSensorsInterface::lidar_timer_callback() {
  // fill the message
  lidar_msg_.header.stamp = this->now();
  lidar_msg_.header.frame_id = "av_lidar";
  lidar_msg_.angle_min = -M_PI;
  lidar_msg_.angle_max = M_PI;
  lidar_msg_.angle_increment = M_PI / 180.0;
  lidar_msg_.time_increment = (1 / 10) / 360;
  lidar_msg_.scan_time = 1 / 10;
  lidar_msg_.range_min = 0.0;
  lidar_msg_.range_max = 10.0;
  lidar_msg_.ranges.resize(360);
  for (size_t i = 0; i < lidar_msg_.ranges.size(); i++) {
    lidar_msg_.ranges[i] = 10.0;
  }
  // publish the message
  lidar_publisher_->publish(lidar_msg_);
}

/**
 * @brief Static function to generate a radar return message
 * TODO: This can be optimized by returning a pointer to the radar return
 * @return radar_msgs::msg::RadarReturn
 */
static radar_msgs::msg::RadarReturn generate_radar_return() {
  auto radar_return = radar_msgs::msg::RadarReturn();
  radar_return.range = 10.0;
  radar_return.amplitude = 1.0;
  radar_return.azimuth = 0.0;
  radar_return.doppler_velocity = 0.0;
  radar_return.elevation = 0.0;
  return radar_return;
}

void AVSensorsInterface::radar_timer_callback() {
  // fill the message
  radar_msg_.header.stamp = this->now();
  radar_msg_.header.frame_id = "av_radar";
  std::vector<radar_msgs::msg::RadarReturn> radar_returns;

  // add two radar returns
  for (size_t i = 0; i < 2; i++) {
    radar_returns.push_back(generate_radar_return());
  }
  radar_msg_.returns = radar_returns;

  // publish the message
  radar_publisher_->publish(radar_msg_);
}

/**
 * @brief Main function to create the AVSensorsInterface node
 *
 * @param argc  number of command line arguments
 * @param argv  command line arguments
 * @return int  0 if successful
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AVSensorsInterface>("av_sensors");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}