#include <rclcpp/rclcpp.hpp>
#include "publisher_node.hpp"

void PublisherNode::timer_callback() {
  msg_.data = "Help me Obi-Wan Kenobi, you are my only hope.";
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << msg_.data);
  publisher_->publish(msg_);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto publisher_node =
      std::make_shared<PublisherNode>("help_pub_cpp");
  rclcpp::spin(publisher_node);
  rclcpp::shutdown();
}