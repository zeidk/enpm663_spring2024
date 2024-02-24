#include <radar_msgs/msg/radar_return.hpp>
#include <rclcpp/rclcpp.hpp>

#include "av_demo/av_actions_interface.hpp"

void AVActionsInterface::data_fusion_callback() {
  // Do sensor fusion with radar_msg_ and lidar_msg_
  RCLCPP_INFO(this->get_logger(), "Data fusion done");
}

/**
 * @brief Main function to create the AVActionsInterface node
 *
 * @param argc  number of command line arguments
 * @param argv  command line arguments
 * @return int  0 if successful
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AVActionsInterface>("av_actions");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}