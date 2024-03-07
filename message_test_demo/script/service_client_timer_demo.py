#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from message_test_demo.service_client_timer_interface import ServiceClientTimerInterface


def main(args=None):
    rclpy.init(args=args)
    node = ServiceClientTimerInterface("client_timer_demo")
    # create a multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()