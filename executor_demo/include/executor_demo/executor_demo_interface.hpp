#pragma once

#include <rclcpp/rclcpp.hpp>

/**
 * @brief  SingleThreadedExecutorInterface class
 * This class is a simple example of a node that uses the single threaded
 *
 */
class SingleThreadedExecutorInterface : public rclcpp::Node {
 public:
  SingleThreadedExecutorInterface(std::string node_name) : Node(node_name) {
    // Timer1
    timer1_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      RCLCPP_INFO(this->get_logger(), "Timer1 callback");
    });
    // Timer1
    timer2_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      RCLCPP_INFO(this->get_logger(), "Timer2 callback");
    });
    // Timer1
    timer3_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
      RCLCPP_INFO(this->get_logger(), "Timer3 callback");
    });
    // Timer1
    timer4_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
      RCLCPP_INFO(this->get_logger(), "Timer4 callback");
    });
  }

 private:
  // timer objects
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::TimerBase::SharedPtr timer3_;
  rclcpp::TimerBase::SharedPtr timer4_;
};

/**
 * @brief  DualMutuallyExclusiveInterface class
 * This class shows how to run two mutually exclusive callback groups
 * in a single node.
 *
 * Timer1 and Timer2 are in the same mutually exclusive group: This means that
 * these two timers will never be called at the same time. Each timer in the
 * same mutually exclusive group will be called in the order they were created.
 * Timer3 and Timer4 are in the same mutually exclusive group: This means that
 * these two timers will never be called at the same time. Each timer in the
 * same mutually exclusive group will be called in the order they were created.
 *
 */
class DualMutuallyExclusiveInterface : public rclcpp::Node {
 public:
  DualMutuallyExclusiveInterface(std::string node_name) : Node(node_name) {
    // Create a mutually exclusive callback group
    group1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, true);
    // Create a mutually exclusive callback group
    group2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, true);

    // Timer1
    timer1_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&DualMutuallyExclusiveInterface::timer1_callback, this),
        group1_);
    // Timer2
    timer2_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DualMutuallyExclusiveInterface::timer2_callback, this),
        group1_);
    // Timer3
    timer3_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DualMutuallyExclusiveInterface::timer3_callback, this),
        group2_);
    // Timer4
    timer4_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DualMutuallyExclusiveInterface::timer4_callback, this),
        group2_);
  }

 private:
  // callback groups
  rclcpp::CallbackGroup::SharedPtr group1_;
  rclcpp::CallbackGroup::SharedPtr group2_;
  // timer objects
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::TimerBase::SharedPtr timer3_;
  rclcpp::TimerBase::SharedPtr timer4_;

  // callback functions
  void timer1_callback() { RCLCPP_INFO(this->get_logger(), "Timer1 callback"); }
  void timer2_callback() { RCLCPP_INFO(this->get_logger(), "Timer2 callback"); }
  void timer3_callback() { RCLCPP_INFO(this->get_logger(), "Timer3 callback"); }
  void timer4_callback() { RCLCPP_INFO(this->get_logger(), "Timer4 callback"); }
};

// DualMutuallyExclusiveInterface ExclusiveReentrantInterface ReentrantInterface