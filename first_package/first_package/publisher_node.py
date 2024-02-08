from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    """
    A ROS2 publisher node that sends string messages to the 'help' topic at 2-second intervals.

    The messages contain a counter and a fixed message, which demonstrates a simple way to publish data continuously in ROS2.

    Attributes:
        _publisher (Publisher): The ROS2 publisher object for sending messages.
        _timer (Timer): A timer object that triggers the publishing event every 2 seconds.
        _msg (String): A `std_msgs/msg/String` message that holds the data to be published.
        _counter (int): A simple counter that increments with each published message, included in the message data.

    Args:
        node_name (str): The name of the node.
    """

    def __init__(self, node_name):
        """
        Initializes the publisher node, creates a publisher for the 'help' topic, and starts a timer to publish messages every 2 seconds.

        Args:
            node_name (str): The name of the node, passed to the parent Node class.
        """
        super().__init__(node_name)
        # Create a publisher object for the 'help' topic with a queue size of 10.
        self._publisher = self.create_publisher(String, "help", 10)
        # Create a timer that calls `timer_callback` every 2 seconds.
        self._timer = self.create_timer(2, self.timer_callback)
        # Initialize the message object that will be published.
        self._msg = String()
        # Initialize the counter to 0.
        self._counter = 0

    def timer_callback(self):
        """
        Callback function for the timer event. This function constructs the message to be published,
        logs the message to the ROS2 logger, and increments the message counter.

        The message format is "<counter>: Help me Obi-Wan Kenobi, you are my only hope.", demonstrating
        a simple message pattern including a dynamic component (the counter).
        """
        # Set the message data.
        self._msg.data = (
            f"{self._counter}: Help me Obi-Wan Kenobi, you are my only hope."
        )
        # Publish the message.
        self._publisher.publish(self._msg)
        # Log the message being published.
        self.get_logger().info(f"Publishing: {self._msg.data}")
        # Increment the counter for the next message.
        self._counter += 1
