from rclpy.node import Node


class AdvancedNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"Hello from {self.get_name()}")


# -- template for a node
# from rclpy.node import Node

# class TemplateNode(Node): # change the class name
#     def __init__(self, node_name):
#         super().__init__(node_name)
#         self.get_logger().info(f"Hello from {self.get_name()}")