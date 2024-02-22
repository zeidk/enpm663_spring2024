from rclpy.node import Node
from sensor_msgs.msg import Image


class AVCamerasInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Publishers for the cameras
        self._camera_pub = self.create_publisher(Image, "av_camera", 10)

        # Messages for the sensors
        self._camera_msg = Image()
        # Timer for the camera running at 15 Hz
        # use miliseconds in the timer

        self._camera_timer = self.create_timer(1 / 15, self.camera_callback)

        self.get_logger().info(f"{node_name} initialized")

    def camera_callback(self):
        """
        Callback function for the camera sensor
        """

        # Populate the Image message
        self._camera_msg.header.stamp = self.get_clock().now().to_msg()
        self._camera_msg.header.frame_id = "av_camera"
        self._camera_msg.height = 480
        self._camera_msg.width = 640
        self._camera_msg.encoding = "rgb8"
        self._camera_msg.is_bigendian = False
        self._camera_msg.step = 640 * 3
        self._camera_msg.data = [0] * 640 * 480 * 3
        # Publish the message
        self.get_logger().info("Publishing camera message")
        self._camera_pub.publish(self._camera_msg)
