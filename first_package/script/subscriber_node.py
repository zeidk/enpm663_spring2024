#!/usr/bin/env python3

import rclpy  # ROS 2 Python client
from first_package.subscriber_node import SubscriberNode


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode("help_sub_py")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
