from rclpy.node import Node
from interface_demo_msgs.msg import VehicleStatus
from interface_demo_msgs.srv import GetSpeedProfile
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ServiceClientSubscriberInterface(Node):
    """
    Class to demonstrate the use of a service client.
    This class consists of two service clients, one synchronous and one asynchronous.
    Both clients are called in the subscriber callback function.
    """

    mutex_group = MutuallyExclusiveCallbackGroup()

    def __init__(self, node_name):
        super().__init__(node_name)

        # Create a subscriber to the vehicle_status topic
        self._subscriber = self.create_subscription(
            VehicleStatus,  # Message type
            "vehicle_status",  # Topic name
            self._vehicle_status_cb,  # Callback function
            100,  # QoS profile,  # Callback group
        )

        # Create a client to call the get_speed_profile service
        # This client will be called synchronously
        # The callbacks associated with this service client will be executed in a mutually exclusive manner, ensuring thread safety.
        # Without this callback group, there will be a deadlock when the service client is called synchronously.
        self._sync_client = self.create_client(
            GetSpeedProfile,  # Service type
            "get_speed_profile",  # Service name
            callback_group=ServiceClientSubscriberInterface.mutex_group,  # Callback group
        )

        # Create a client to call the get_speed_profile service
        # This client will be called asynchronously
        # A callback group is not needed
        self._async_client = self.create_client(GetSpeedProfile, "get_speed_profile")

        # Wait for the service to be available
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Create a request object
        self._request = GetSpeedProfile.Request()
        # Sanity check
        self._logger.info("client created")

    def _vehicle_status_cb(self, msg: VehicleStatus):
        """
        Callback function for the subscriber.
        This function is called whenever a message is received on the vehicle_status topic.
        """

        # Call the service clients
        self._send_async_request(float(msg.speed))
        self._send_sync_request(float(msg.speed))

    def _send_async_request(self, speed: float) -> None:
        """
        Function to send an asynchronous request to the service.

        Args:
            speed (float): The speed value to be sent in the request
        """

        # Wait for the service to be available
        while not self._async_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._request.speed = float(speed)

        # Call the service asynchronously
        future = self._async_client.call_async(self._request)

        # Add a callback function to be called when the future is complete
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        """
        Callback function for the future object

        Args:
            future (Future): A future object
        """
        self.get_logger().info(f"🔴 TimerAsyncResult: {future.result().status}")

    def _send_sync_request(self, speed: float) -> None:
        """
        Send a request synchronously to the server

        Args:
            speed (float): The speed value to be sent in the request
        """

        # Wait for the service to be available
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._request.speed = float(speed)

        response = self._sync_client.call(self._request)
        self.get_logger().info(f"🔵 TimerSyncResult: {response.status}")
