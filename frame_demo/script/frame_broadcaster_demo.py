#!/usr/bin/env python3

import rclpy
from frame_demo.frame_demo_interface import BroadcasterDemo


def main(args=None):
    rclpy.init(args=args)
    broadcaster_node = BroadcasterDemo("broadcaster_demo")

    try:
        rclpy.spin(broadcaster_node)
    except KeyboardInterrupt:
        broadcaster_node.get_logger().info("KeyboardInterrupt, shutting down.\n")
    broadcaster_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
