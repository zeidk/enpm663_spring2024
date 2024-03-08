#!/usr/bin/env python3

import rclpy
from message_test_demo.service_server_interface import ServiceServerInterface


def main(args=None):
    rclpy.init(args=args)
    node = ServiceServerInterface("service_server_demo")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
