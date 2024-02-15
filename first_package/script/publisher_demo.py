#!/usr/bin/env python3

"""
This script initializes a ROS2 node using rclpy, which publishes messages using a custom PublisherNode.
The PublisherNode is defined in the first_package package, and it is responsible for publishing messages
to a topic as defined within its implementation. This script is an entry point for running the publisher node,
setting it up, spinning it to keep it alive and processing data, and properly shutting it down afterwards.

Usage:
    To run this script, use the following command in a terminal:
    ```
    ros2 run first_package publisher_demo.py
    ```
"""
# Import ROS Client Library for Python
import rclpy

# Import the custom PublisherNode class
from first_package.publisher_interface import (
    PublisherNode,
)


def main(args=None):
    """
    Main function to initialize and run the ROS2 publisher node.

    Args:
        args (list, optional): Command-line arguments passed to the node. Defaults to None.
    """
    rclpy.init(args=args)  # Initialize the ROS client library
    node = PublisherNode("publisher_py")  # Create an instance of the PublisherNode
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()  # Execute the main function when the script is run
