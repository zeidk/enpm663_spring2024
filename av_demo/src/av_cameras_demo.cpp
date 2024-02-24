#include <radar_msgs/msg/radar_return.hpp>
#include <rclcpp/rclcpp.hpp>

#include "av_demo/av_cameras_interface.hpp"

void AVCamerasInterface::camera_timer_callback() {
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

/**
 * @brief Main function to create the AVSensorsInterface node
 *
 * @param argc  number of command line arguments
 * @param argv  command line arguments
 * @return int  0 if successful
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AVCamerasInterface>("av_cameras");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}