#include <rclcpp/rclcpp.hpp>

#include "advanced_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdvancedNode>("echo_node_cpp");
  rclcpp::spin(node);
  rclcpp::shutdown();
}