from rclpy.node import Node
from interface_demo_msgs.msg import VehicleStatus
from interface_demo_msgs.srv import GetSpeedProfile
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ServiceClientTimerInterface(Node):
    """
    Class to demonstrate the use of a service client.
    This class consists of two service clients, one synchronous and one asynchronous.
    Both clients are called in the timer callback function.
    """

    mutex_group1 = MutuallyExclusiveCallbackGroup()
    mutex_group2 = MutuallyExclusiveCallbackGroup()

    def __init__(self, node_name):
        super().__init__(node_name)

        # This attribute is set in the subscriber callback function
        # and is used in the timer callback function
        self._current_vehicle_speed = False

        # Create various callback groups to ensure that various events are called in a mutually exclusive manner
        # timer_cb_group = MutuallyExclusiveCallbackGroup()
        # subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        # Create a subscriber to the vehicle_status topic
        self._subscriber = self.create_subscription(
            VehicleStatus,  # Message type
            "vehicle_status",  # Topic name
            self._vehicle_status_cb,  # Callback function
            100,  # QoS profile
            callback_group=ServiceClientTimerInterface.mutex_group1,  # Callback group
        )

        self._timer_async_call = self.create_timer(
            1,  # Timer period in seconds
            self._timer_async_call_cb,  # Callback function
            callback_group=ServiceClientTimerInterface.mutex_group1,  # Callback group
        )

        # Create a timer to call the service synchronously every second
        self._timer_sync_call = self.create_timer(
            1,  # Timer period in seconds
            self._timer_sync_call_cb,  # Callback function
            callback_group=ServiceClientTimerInterface.mutex_group1,  # Callback group
        )

        # Create a client to call the get_speed_profile service
        # This client will be called synchronously
        # The callback group ensures that the callback function is called in a mutually exclusive manner
        self._sync_client = self.create_client(
            GetSpeedProfile,  # Service type
            "get_speed_profile",  # Service name
            callback_group=ServiceClientTimerInterface.mutex_group2,  # Callback group
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

        self._logger.info("client created")

    def _vehicle_status_cb(self, msg: VehicleStatus):
        """
        Callback function for the subscriber.
        This function is called whenever a message is received on the vehicle_status topic.
        """
        # Store the current vehicle speed
        self._current_vehicle_speed = msg.speed

    def _timer_sync_call_cb(self):
        """
        Callback function for a timer which calls the service synchronously.
        This function is called every second.
        """

        # Call the synchronous service client
        self._send_sync_request()

    def _timer_async_call_cb(self):
        """
        Callback function for a timer which calls the service asynchronously.
        This function is called every second.
        """
        # Call the asynchronous service client
        self._send_async_request()

    def _send_async_request(self):
        """
        Function to send an asynchronous request to the service.
        """
        while not self._async_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._request.speed = float(self._current_vehicle_speed)

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

    def _send_sync_request(self):
        """
        Send a request synchronously to the server
        """
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._request.speed = float(self._current_vehicle_speed)

        response = self._sync_client.call(self._request)
        self.get_logger().info(f"🔵 TimerSyncResult: {response.status}")
