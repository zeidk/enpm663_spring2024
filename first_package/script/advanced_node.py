#!/usr/bin/env python3

import rclpy
from first_package.advanced_node import AdvancedNode


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedNode("echo_node_py")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
