#include <rclcpp/rclcpp.hpp>
#include <random>
#include "message_test_demo/message_test_demo_interface.hpp"

void MessageTestDemoInterface::timer_callback() {
  // generate a random float number between 20 and 80
  // Set up random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(20.0, 80.0);

    // Generate random double between 20 and 80
    double random_double = dis(gen);

  // update the message
  msg_.name = "vehicle";
  msg_.speed = random_double;
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