from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from radar_msgs.msg import RadarReturn, RadarScan
import random
import time
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class AVSensorsInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Publishers for the sensors
        self._lidar_pub = self.create_publisher(LaserScan, "av_lidar", 10)
        self._camera_pub = self.create_publisher(Image, "av_camera", 10)
        self._radar_pub = self.create_publisher(RadarScan, "av_radar", 10)

        # Messages for the sensors
        self._lidar_msg = LaserScan()
        self._camera_msg = Image()
        self._radar_msg = RadarScan()

        # Timers for the sensors

        # Timer for the lidar running at 10 Hz
        self._lidar_timer = self.create_timer(1 / 10, self.lidar_callback)
        # Timer for the camera running at 15 Hz
        self._camera_timer = self.create_timer(1 / 15, self.camera_callback)
        # Timer for the radar running at 20 Hz
        self._radar_timer = self.create_timer(1 / 20, self.radar_callback)

        self.get_logger().info(f"{node_name} initialized")

    def lidar_callback(self):
        """
        Callback function for the lidar sensor
        """

        # Populate the LaserScan message
        self._lidar_msg.header.stamp = self.get_clock().now().to_msg()
        self._lidar_msg.header.frame_id = "av_lidar"
        self._lidar_msg.angle_min = -3.14159
        self._lidar_msg.angle_max = 3.14159
        self._lidar_msg.angle_increment = 0.0174533
        self._lidar_msg.time_increment = 0.0
        self._lidar_msg.scan_time = 0.1
        self._lidar_msg.range_min = 0.0
        self._lidar_msg.range_max = 100.0
        self._lidar_msg.ranges = [1.0] * 360
        self._lidar_msg.intensities = [1.0] * 360
        # Publish the message
        # self.get_logger().info("Publishing lidar message")
        self._lidar_pub.publish(self._lidar_msg)

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
        # self.get_logger().info("Publishing camera message")
        self._camera_pub.publish(self._camera_msg)

    def generate_random_radar_return(self):
        """
        Generate a random radar return message

        Returns:
            : _description_
        """
        radar_return = RadarReturn()
        radar_return.range = random.uniform(
            0.0, 100.0
        )  # Random range between 0 and 100 meters
        radar_return.azimuth = random.uniform(
            -180.0, 180.0
        )  # Random azimuth between -180 and 180 degrees
        radar_return.elevation = random.uniform(
            -90.0, 90.0
        )  # Random elevation between -90 and 90 degrees
        radar_return.doppler_velocity = random.uniform(
            -50.0, 50.0
        )  # Random Doppler velocity between -50 and 50 m/s
        radar_return.amplitude = random.uniform(
            0.0, 100.0
        )  # Random amplitude between 0 and 100
        return radar_return

    def radar_callback(self):
        """
        Callback function for the radar sensor
        """

        # Create an array of RadarReturn messages

        radar_returns = []
        for _ in range(5):  # Generate 5 random radar returns
            radar_returns.append(self.generate_random_radar_return())

        # Populate the RadarScan message
        self._radar_msg.header.stamp = self.get_clock().now().to_msg()
        self._radar_msg.header.frame_id = "av_radar"
        self._radar_msg.returns = radar_returns
        # Publish the message
        # self.get_logger().info("Publishing radar message")
        self._radar_pub.publish(self._radar_msg)
