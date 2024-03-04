from rclpy.node import Node


class ExecutorInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Timer1
        self._timer1 = self.create_timer(2, self.timer1_callback)
        # Timer2
        self._timer2 = self.create_timer(2, self.timer2_callback)
        # Timer3
        self._timer3 = self.create_timer(2, self.timer3_callback)
        # Timer4
        self._timer4 = self.create_timer(2, self.timer4_callback)

        self.get_logger().info(f"{node_name} initialized")


    def timer1_callback(self):
        """
        Callback function for timer1
        """
        self.get_logger().info("⭐Timer1 callback")

    def timer2_callback(self):
        """
        Callback function for timer2
        """
        self.get_logger().info("🟦Timer2 callback")
        
    def timer3_callback(self):
        """
        Callback function for timer3
        """
        self.get_logger().info("🔶Timer3 callback")
        
    def timer4_callback(self):
        """
        Callback function for timer4
        """
        self.get_logger().info("🔴Timer4 callback")