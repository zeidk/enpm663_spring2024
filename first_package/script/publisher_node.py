#!/usr/bin/env python3

import rclpy  # ROS 2 Python client
from first_package.publisher_node import PublisherNode


def main(args=None):
    # 1. Initialize ROS communications for a given context
    rclpy.init(args=args)
    # 2. Instantiate a Node
    node = PublisherNode("help_pub_py")
    rclpy.spin(node)
    # 3. Shutdown a previously initialized context
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
