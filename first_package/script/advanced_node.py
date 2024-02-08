#!/usr/bin/env python3

"""
This script initializes and runs a ROS2 node using rclpy. The node implemented in this script
is `AdvancedNode` from the `first_package` package. The script demonstrates initializing the ROS2
Python client library, creating an instance of `AdvancedNode`, and spinning the node to handle
callbacks.

Usage:
    This script is intended to be run from the command line. It does not take any command-line arguments.
    To run the script, use the following command:

    $ ros2 run first_package advanced_node.py
"""

import rclpy  # Import ROS Client Library for Python
from first_package.advanced_node import AdvancedNode  # Import the AdvancedNode class


def main(args=None):
    """
    Main function for initializing and running the ROS2 node.

    Args:
        args (list, optional): List of arguments passed to the script. Defaults to None.
    """
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    node = AdvancedNode(
        "echo_node_py"
    )  # Create an instance of AdvancedNode named "echo_node_py"
    rclpy.spin(node)  # Keep the node alive to listen for callbacks
    rclpy.shutdown()  # Shutdown the ROS2 Python client library on node termination


if __name__ == "__main__":
    main()  # Execute the main function if the script is run directly
