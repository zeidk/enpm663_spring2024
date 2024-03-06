#include <rclcpp/rclcpp.hpp>

#include "message_test_demo/message_test_demo_interface.hpp"

void MessageTestDemoInterface::timer_callback() {
// generate a random float number between 20 and 80
  float random_speed = 20 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (80 - 20)));

  // update the message
  msg_.name = "vehicle";
  msg_.speed = random_speed;
  msg_.pose.x = 1.0;
  msg_.pose.y = 2.0;
  msg_.pose.theta = 3.0;

  // publish the message
  publisher_->publish(msg_);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MessageTestDemoInterface>("message_test_demo");
  rclcpp::spin(node);
  rclcpp::shutdown();
}