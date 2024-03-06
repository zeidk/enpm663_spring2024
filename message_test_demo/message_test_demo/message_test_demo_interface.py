from rclpy.node import Node
from interface_demo_msgs.msg import VehicleStatus
import random


class MessageTestDemoInterface(Node):
    """
    Class to publish vehicle status
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self._pub = self.create_publisher(VehicleStatus, "vehicle_status", 10)
        self._timer = self.create_timer(4, self._timer_callback)
        self._msg = VehicleStatus()

    def _timer_callback(self):
        # generate a random float number for speed between 20 and 80
        random_speed = random.uniform(20, 80)
        self._msg.speed = random_speed
        self._msg.name = "vehicle"
        self._msg.pose.x = 1.0
        self._msg.pose.y = 2.0
        self._msg.pose.theta = 3.0

        self._pub.publish(self._msg)