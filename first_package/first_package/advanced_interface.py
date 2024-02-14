from rclpy.node import Node


class AdvancedNode(Node):
    """
    A custom ROS2 Node class that extends the basic Node class provided by rclpy.

    This class initializes a ROS2 node and logs a greeting message upon creation. It serves as an example of how to create a custom node class for more advanced ROS2 applications.

    Attributes:
        None explicitly defined beyond those inherited from the rclpy.node.Node class.

    Args:
        node_name (str): The name of the node. This name is used by ROS2 for node identification.
    """

    def __init__(self, node_name):
        """
        Initializes the AdvancedNode instance, sets up the node with the given name, and logs a greeting message.

        Args:
            node_name (str): The name of the node, passed to the parent Node class's constructor.
        """
        super().__init__(node_name)  
        # Log a greeting message indicating successful node creation.
        self.get_logger().info(f"Hello from {self.get_name()}")