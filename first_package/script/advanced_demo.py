#!/usr/bin/env python3

"""
This script initializes and runs a ROS2 node using rclpy. The node implemented in this script
is `AdvancedNode` from the `first_package` package. The script demonstrates initializing the ROS2
Python client library, creating an instance of `AdvancedNode`, and spinning the node to handle
callbacks.

Usage:
    This script is intended to be run from the command line. It does not take any command-line arguments.
    To run the script, use the following command:

    $ ros2 run first_package advanced_demo.py
"""
# Import ROS Client Library for Python
import rclpy

# Import the AdvancedNode class
from first_package.advanced_interface import (
    AdvancedNode,
)


def main(args=None):
    """
    Main function for initializing and running the ROS2 node.

    Args:
        args (list, optional): List of arguments passed to the script. Defaults to None.
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    # Create an instance of AdvancedNode named "advanced_py"
    node = AdvancedNode("advanced_py")
    # Keep the node alive to listen for callbacks
    # Note: This function is not necessary for the current implementation, but it is included for demonstration purposes
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()  # Execute the main function if the script is run directly
