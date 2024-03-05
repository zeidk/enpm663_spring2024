#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from executor_demo.executor_demo_interface import ReentrantInterface


def main(args=None):
    rclpy.init(args=args)
    node = ReentrantInterface("reentrant_demo")

    # Use a MultiThreadedExecutor to allow callbacks in different groups to run concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()