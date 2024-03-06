#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "executor_demo/executor_demo_interface.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<ExclusiveReentrantInterface>("exclusive_reentrant_demo");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();  // This will start the execution
  rclcpp::shutdown();
}
