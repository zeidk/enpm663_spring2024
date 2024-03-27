#!/usr/bin/env python3

import rclpy
from frame_demo.frame_demo_interface import KDLFrameDemo


def main(args=None):
    rclpy.init(args=args)
    kdl_frame = KDLFrameDemo("kdl_frame_demo")

    try:
        rclpy.spin(kdl_frame)
    except KeyboardInterrupt:
        kdl_frame.get_logger().info("KeyboardInterrupt, shutting down.\n")
    kdl_frame.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
