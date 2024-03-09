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
      RCLCPP_INFO(this->get_logger(), "\033[1;33m Timer1 callback\033[0m");
    });
    // Timer2
    timer2_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      RCLCPP_INFO(this->get_logger(), "\033[1;34m Timer2 callback\033[0m");
    });
    // Timer3
    timer3_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      RCLCPP_INFO(this->get_logger(), "\033[1;32m Timer3 callback\033[0m");
    });
    // Timer4
    timer4_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      RCLCPP_INFO(this->get_logger(), "\033[1;31m Timer4 callback\033[0m");
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
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // Create a mutually exclusive callback group
    group2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Timer1
    timer1_ = this->create_wall_timer(
        std::chrono::seconds(1),
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
  void timer1_callback() {
    // while (1) {
    //   // Do nothing
    // }
    RCLCPP_INFO(this->get_logger(), "\033[1;33m Timer1 callback\033[0m");
  }
  void timer2_callback() {
    RCLCPP_INFO(this->get_logger(), "\033[1;34m Timer2 callback\033[0m");
  }
  void timer3_callback() {
    // while (1) {
    //   // Do nothing
    // }
    RCLCPP_INFO(this->get_logger(), "\033[1;32m Timer3 callback\033[0m");
  }
  void timer4_callback() {
    RCLCPP_INFO(this->get_logger(), "\033[1;31m Timer4 callback\033[0m");
  }
};

class ExclusiveReentrantInterface : public rclcpp::Node {
 public:
  ExclusiveReentrantInterface(std::string node_name) : Node(node_name) {
    // Create a mutually exclusive callback group
    group1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // Create a reentrant callback group
    group2_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Timer1
    timer1_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ExclusiveReentrantInterface::timer1_callback, this),
        group1_);
    // Timer2
    timer2_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ExclusiveReentrantInterface::timer2_callback, this),
        group1_);
    // Timer3
    timer3_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ExclusiveReentrantInterface::timer3_callback, this),
        group2_);
    // Timer4
    timer4_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ExclusiveReentrantInterface::timer4_callback, this),
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
  void timer1_callback() {
    // while (1) {
    //   // Do nothing
    // }
    RCLCPP_INFO(this->get_logger(), "\033[1;33m Timer1 callback\033[0m");
  }
  void timer2_callback() {
    RCLCPP_INFO(this->get_logger(), "\033[1;34m Timer2 callback\033[0m");
  }
  void timer3_callback() {
    // while (1) {
    //   // Do nothing
    // }
    RCLCPP_INFO(this->get_logger(), "\033[1;32m Timer3 callback\033[0m");
  }
  void timer4_callback() {
    RCLCPP_INFO(this->get_logger(), "\033[1;31m Timer4 callback\033[0m");
  }
};

class ReentrantInterface : public rclcpp::Node {
 public:
  ReentrantInterface(std::string node_name) : Node(node_name) {
    // Create a reentrant callback group
    group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Timer1
    timer1_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ReentrantInterface::timer1_callback, this), group_);
    // Timer2
    timer2_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ReentrantInterface::timer2_callback, this), group_);
    // Timer3
    timer3_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ReentrantInterface::timer3_callback, this), group_);
    // Timer4
    timer4_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ReentrantInterface::timer4_callback, this), group_);
  }

 private:
  // callback groups
  rclcpp::CallbackGroup::SharedPtr group_;
  // timer objects
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::TimerBase::SharedPtr timer3_;
  rclcpp::TimerBase::SharedPtr timer4_;

  // callback functions
  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "\033[1;33m Timer1 callback\033[0m");
  }
  void timer2_callback() {
    RCLCPP_INFO(this->get_logger(), "\033[1;34m Timer2 callback\033[0m");
  }
  void timer3_callback() {
    RCLCPP_INFO(this->get_logger(), "\033[1;32m Timer3 callback\033[0m");
  }
  void timer4_callback() {
    RCLCPP_INFO(this->get_logger(), "\033[1;31m Timer4 callback\033[0m");
  }
};
