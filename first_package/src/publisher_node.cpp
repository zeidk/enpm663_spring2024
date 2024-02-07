#include "first_package/publisher_node.hpp"

#include <rclcpp/rclcpp.hpp>

void PublisherNode::timer_callback() {
  // convert counter to string
  // concatenate the strings
  msg_.data = "Help me Obi-Wan Kenobi, you are my only hope.";
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << msg_.data);
  publisher_->publish(msg_);
  counter_++;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto publisher_node = std::make_shared<PublisherNode>("help_pub_cpp");
  rclcpp::spin(publisher_node);
  rclcpp::shutdown();
}