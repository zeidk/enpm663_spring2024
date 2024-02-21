from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from radar_msgs.msg import RadarScan


class AVActionsInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Sensor subscribers
        self._lidar_sub = self.create_subscription(
            LaserScan, "av_lidar", self.lidar_callback, 10
        )
        self._camera_sub = self.create_subscription(
            Image, "av_camera", self.camera_callback, 10
        )
        self._radar_sub = self.create_subscription(
            RadarScan, "av_radar", self.radar_callback, 10
        )

        # Messages for the sensors
        self._lidar_msg = LaserScan()
        self._camera_msg = Image()
        self._radar_msg = RadarScan()

        # Timer for data fusion running at 30 Hz
        # The callback fuses data from the lidar and the radar for obstacle detection
        self._data_fusion_timer = self.create_timer(1 / 30, self.data_fusion_callback)

        self.get_logger().info(f"{node_name} initialized")

    def lidar_callback(self, msg):
        """
        Callback function for the lidar sensor
        """
        self._lidar_msg = msg
        self.get_logger().info("Lidar data received")

    def camera_callback(self, msg):
        """
        Callback function for the camera sensor
        """
        self._camera_msg = msg
        self.get_logger().info("Camera data received")

    def radar_callback(self, msg):
        """
        Callback function for the radar sensor
        """
        self._radar_msg = msg
        self.get_logger().info("Radar data received")

    def data_fusion_callback(self):
        """
        Callback function for data fusion
        """
        # self.get_logger().info("Data fusion callback")
        # Perform data fusion here
        # For now, just print the data
        # self.get_logger().info(f"Lidar data: {self._lidar_msg}")
        # self.get_logger().info(f"Radar data: {self._radar_msg}")
        # Do sensor fusion here
        self.get_logger().info("Data fusion complete")
