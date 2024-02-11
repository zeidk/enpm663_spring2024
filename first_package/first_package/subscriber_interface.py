from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    """A ROS2 subscriber node that listens to string messages on a specified topic.

    This class creates a node that subscribes to the "help" topic, expecting messages of type `std_msgs/msg/String`.
    It logs the received messages to the ROS2 logger.

    Attributes:
        _subscriber (rclpy.subscription.Subscription): The subscription object for receiving string messages.

    Args:
        node_name (str): The name of the node, provided during instantiation.
    """

    def __init__(self, node_name):
        """Initialize the SubscriberNode with a given name and create a subscription to the "help" topic.

        The method sets up a subscriber for the "help" topic with a string message type. It specifies
        a callback function that is invoked upon receiving a message.

        Args:
            node_name (str): The name of the node.
        """
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
