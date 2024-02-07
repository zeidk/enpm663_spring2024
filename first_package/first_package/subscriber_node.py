import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):

    """SubscriberNode This class is a node that subscribes to a string message and publishes the first word of the message.

    Arguments:
        Node -- Node class from rclpy
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self._subscriber = self.create_subscription(
            String, "help", self.subscriber_callback, 10
        )

    def subscriber_callback(self, msg):
        """subscriber_callback Callback function for the subscriber to the chatter663 topic.

        Arguments:
            msg -- String message from the chatter663 topic.
        """

        self.get_logger().info(f"Receiving: {msg.data}")
