#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


def main(args=None):
    """
    Main function to initialize a ROS2 node and spin it.

    This function initializes the ROS2 client library, creates a node, logs a greeting message,
    and keeps the node alive until it's manually terminated or receives a shutdown signal.

    Args:
        args (list, optional): A list of command-line arguments passed to the ROS2 node. Defaults to None.
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a new node with the name 'echo_node_py'
    node = Node("echo_node_py")

    # Log a greeting message using the node's logger
    node.get_logger().info(f"Hello from {node.get_name()}")

    # Keep the node alive (listening for callbacks) until it's shut down
    rclpy.spin(node)

    # Shutdown the ROS2 Python client library to clean up resources
    rclpy.shutdown()


if __name__ == '__main__':
    main()