from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._subscriber = self.create_subscription(
            String, "leia", self.receive_message, 10
        )
        # The queue size is set to 10, which is the size of the message queue.

    def receive_message(self, msg):
        """Handle incoming messages on the "help" topic.

        This function is called when a new message is received on the "help" topic. It logs the
        message content using the node's logger.

        Args:
            msg (std_msgs.msg.String): The received message object, containing the string data.
        """
        self.get_logger().info(f"Receiving: {msg.data}")
        # Logs the received message data.
