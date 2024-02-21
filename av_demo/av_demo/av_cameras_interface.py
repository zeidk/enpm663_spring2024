from rclpy.node import Node
from sensor_msgs.msg import Image


class AVCamerasInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Publishers for the cameras
        self._camera1_pub = self.create_publisher(Image, "av_camera1", 10)
        self._camera2_pub = self.create_publisher(Image, "av_camera2", 10)
        self._camera3_pub = self.create_publisher(Image, "av_camera3", 10)

        # Messages for the sensors
        self._camera1_msg = Image()
        self._camera2_msg = Image()
        self._camera3_msg = Image()

        # Timer for the camera running at 15 Hz
        # use miliseconds in the timer

        self._camera1_timer = self.create_timer(1 / 15, self.camera1_callback)
        self._camera2_timer = self.create_timer(1 / 15, self.camera2_callback)
        self._camera3_timer = self.create_timer(1 / 15, self.camera3_callback)

        self.get_logger().info(f"{node_name} initialized")

    def camera1_callback(self):
        """
        Callback function for the camera sensor
        """

        # Populate the Image message
        self._camera1_msg.header.stamp = self.get_clock().now().to_msg()
        self._camera1_msg.header.frame_id = "av_camera1"
        self._camera1_msg.height = 480
        self._camera1_msg.width = 640
        self._camera1_msg.encoding = "rgb8"
        self._camera1_msg.is_bigendian = False
        self._camera1_msg.step = 640 * 3
        self._camera1_msg.data = [0] * 640 * 480 * 3
        # Publish the message
        self.get_logger().info("Publishing camera message")
        self._camera1_pub.publish(self._camera1_msg)

    def camera2_callback(self):
        """
        Callback function for the camera sensor
        """

        # Populate the Image message
        self._camera2_msg.header.stamp = self.get_clock().now().to_msg()
        self._camera2_msg.header.frame_id = "av_camera2"
        self._camera2_msg.height = 480
        self._camera2_msg.width = 640
        self._camera2_msg.encoding = "rgb8"
        self._camera2_msg.is_bigendian = False
        self._camera2_msg.step = 640 * 3
        self._camera2_msg.data = [0] * 640 * 480 * 3
        # Publish the message
        # self.get_logger().info("Publishing camera message")
        self._camera2_pub.publish(self._camera2_msg)

    def camera3_callback(self):
        """
        Callback function for the camera sensor
        """

        # Populate the Image message
        self._camera3_msg.header.stamp = self.get_clock().now().to_msg()
        self._camera3_msg.header.frame_id = "av_camera3"
        self._camera3_msg.height = 480
        self._camera3_msg.width = 640
        self._camera3_msg.encoding = "rgb8"
        self._camera3_msg.is_bigendian = False
        self._camera3_msg.step = 640 * 3
        self._camera3_msg.data = [0] * 640 * 480 * 3
        # Publish the message
        # self.get_logger().info("Publishing camera message")
        self._camera3_pub.publish(self._camera3_msg)
