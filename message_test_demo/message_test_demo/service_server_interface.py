from rclpy.node import Node
from interface_demo_msgs.srv import GetSpeedProfile

class Color:
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    RESET = "\033[0m"


class ServiceServerInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.srv = self.create_service(
            GetSpeedProfile, "get_speed_profile", self.handle_request
        )

    def handle_request(self, request, response):
        speed = request.speed
        if speed < 30:
            response.status = GetSpeedProfile.Request.SLOW
        elif speed >= 30 and speed < 60:
            response.status = GetSpeedProfile.Request.NORMAL
        elif speed >= 60:
            response.status = GetSpeedProfile.Request.FAST
            
        self.get_logger().info(
            Color.YELLOW + f"Incoming request: {speed}" + Color.RESET
        )
        self.get_logger().info(
            Color.GREEN + f"Sending response: {response.status}" + Color.RESET
        )
        return response

