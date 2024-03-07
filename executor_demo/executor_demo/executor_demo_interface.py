from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class SingleThreadedExecutorInterface(Node):
    """
    Class to demonstrate the use of a single-threaded executor.
    """
    def __init__(self, node_name):
        super().__init__(node_name)

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback)

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
        # while True:
        #     pass

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


class DualMutuallyExclusiveInterface(Node):
    """
    Class to demonstrate the use of a dual mutually exclusive callback groups.
    """
    def __init__(self, node_name):
        super().__init__(node_name)

        group1 = MutuallyExclusiveCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback, callback_group=group1)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback, callback_group=group1)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback, callback_group=group2)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback, callback_group=group2)

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
        while True:
            pass

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


class ExclusiveReentrantInterface(Node):
    """
    Class to demonstrate the use of an exclusive and reentrant callback groups.
    """
    def __init__(self, node_name):
        super().__init__(node_name)

        group1 = MutuallyExclusiveCallbackGroup()
        group2 = ReentrantCallbackGroup()

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback, callback_group=group1)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback, callback_group=group1)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback, callback_group=group2)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback, callback_group=group2)

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
        

class ReentrantInterface(Node):
    """
    Class to demonstrate the use of a reentrant callback group.
    """
    def __init__(self, node_name):
        super().__init__(node_name)

        group = ReentrantCallbackGroup()

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback, callback_group=group)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback, callback_group=group)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback, callback_group=group)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback, callback_group=group)

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