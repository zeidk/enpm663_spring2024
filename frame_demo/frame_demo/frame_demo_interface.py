import math
from typing import Tuple
import rclpy
import PyKDL
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ariac_msgs.msg import (
    AdvancedLogicalCameraImage as AriacAdvancedLogicalCameraImage,
    Part as AriacPart,
)
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter


def rpy_from_quaternion(quaternion: Quaternion) -> Tuple[float, float, float]:
    """
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        quaternion (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: A tuple containing roll, pitch, and yaw
    """

    rotation = PyKDL.Rotation.Quaternion(
        quaternion.x, quaternion.y, quaternion.z, quaternion.w
    )
    return rotation.GetRPY()


def quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """
    Converts euler roll, pitch, yaw to a tuple of quaternion values (JPL convention).

    Returns:
        Quaternion: tuple of quaternion values (JPL convention)
    """
    quaternion = PyKDL.Rotation.RPY(roll, pitch, yaw).GetQuaternion()

    # Other way to do it
    # cy = math.cos(yaw * 0.5)
    # sy = math.sin(yaw * 0.5)
    # cp = math.cos(pitch * 0.5)
    # sp = math.sin(pitch * 0.5)
    # cr = math.cos(roll * 0.5)
    # sr = math.sin(roll * 0.5)

    # quaternion = [0] * 4
    # quaternion[0] = cy * cp * cr + sy * sp * sr
    # quaternion[1] = cy * cp * sr - sy * sp * cr
    # quaternion[2] = sy * cp * sr + cy * sp * cr
    # quaternion[3] = sy * cp * cr - cy * sp * sr

    return quaternion


# class KDLFrameDemo(Node):
#     """
#     Class showing how to use KDL Frame to compute the pose of a part in the 'world' frame.
#     """

#     def __init__(self, node_name):
#         super().__init__(node_name)

#         self.get_logger().info("KDL demo started")

#         self._camera_pose_in_world = self.set_camera_pose_in_world()
#         self._part_pose_in_camera = self.set_part_pose_in_camera()
#         self._part_pose_in_world = self.multiply_pose(
#             self._camera_pose_in_world, self._part_pose_in_camera
#         )

#         self._rpy = rpy_from_quaternion(self._part_pose_in_world.orientation)

#         self.get_logger().info(
#             f"Part position in world frame: \n x: {self._part_pose_in_world.position.x}, y: {self._part_pose_in_world.position.y}, z: {self._part_pose_in_world.position.z}"
#         )
#         self.get_logger().info(
#             f"Part orientation in world frame: \n rpy: {self._rpy[0]}, {self._rpy[1]}, {self._rpy[2]}"
#         )

#     def set_camera_pose_in_world(self):
#         """
#         Set the pose of the camera in the world frame.
#         Information about the camera pose can be obtained in two different ways:

#             - From the sensors.yaml file
#             - By clicking on the camera in gazebo.

#         Returns:
#             Pose: The pose of the camera in the world frame
#         """

#         # Create a pose object for right_bins_camera
#         pose = Pose()
#         pose.position.x = -2.286
#         pose.position.y = 2.96
#         pose.position.z = 1.8

#         quat_x, quat_y, quat_z, quat_w = quaternion_from_euler(math.pi, math.pi / 2, 0)
#         pose.orientation.x = quat_x
#         pose.orientation.y = quat_y
#         pose.orientation.z = quat_z
#         pose.orientation.w = quat_w

#         return pose

#     def set_part_pose_in_camera(self):
#         """
#         Set the pose of the part detected by the camera.
#         Information about the part pose should be obtained from the camera subscriber.
#         Here we are hardcoding this information for the sake of simplicity.
#         The hardcoded pose was retrieved directly from gazebo.

#         Returns:
#             Pose: The pose of the part in the camera frame
#         """
#         pose = Pose()
#         pose.position.x = 1.0769784427063858
#         pose.position.y = 0.15500024548461805
#         pose.position.z = -0.5660066794253416

#         pose.orientation.x = -0.0013258319361092675
#         pose.orientation.y = -0.7083612841685344
#         pose.orientation.z = -0.0013332542580505244
#         pose.orientation.w = 0.7058475442288268

#         return pose

#     def multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose:
#         """
#         Use KDL to multiply two poses together.
#         Args:
#             p1 (Pose): Pose of the first frame
#             p2 (Pose): Pose of the second frame
#         Returns:
#             Pose: Pose of the resulting frame
#         """

#         orientation1 = pose1.orientation
#         frame1 = PyKDL.Frame(
#             PyKDL.Rotation.Quaternion(
#                 orientation1.x, orientation1.y, orientation1.z, orientation1.w
#             ),
#             PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z),
#         )

#         orientation2 = pose2.orientation
#         frame2 = PyKDL.Frame(
#             PyKDL.Rotation.Quaternion(
#                 orientation2.x, orientation2.y, orientation2.z, orientation2.w
#             ),
#             PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z),
#         )

#         frame3 = frame1 * frame2

#         # return the resulting pose from frame3
#         pose = Pose()
#         pose.position.x = frame3.p.x()
#         pose.position.y = frame3.p.y()
#         pose.position.z = frame3.p.z()

#         q = frame3.M.GetQuaternion()
#         pose.orientation.x = q[0]
#         pose.orientation.y = q[1]
#         pose.orientation.z = q[2]
#         pose.orientation.w = q[3]

#         return pose


class ListenerDemo(Node):
    """
    Class to listen to the frames broadcast by the broadcaster.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        # Get the listen parameter
        self._listen = (
            self.declare_parameter("listen", False).get_parameter_value().bool_value
        )

        # Parameter to choose the parent frame
        self._parent_frame = (
            self.declare_parameter("parent_frame", "world")
            .get_parameter_value()
            .string_value
        )

        # Parameter to choose the child frame
        self._child_frame = (
            self.declare_parameter("child_frame", "first_dynamic_frame")
            .get_parameter_value()
            .string_value
        )

        # Do not execute the demo if listen is false
        if not self._listen:
            self.get_logger().warn("Listener demo is not started.")
            return

        self.get_logger().info("Listener demo started")

        # Create a transform buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Listen to the transform between frames periodically
        self._listener_timer = self.create_timer(1, self._listener_cb)

    def _listener_cb(self):
        """
        Callback function for the listener timer.
        """
        try:
            # Get the transform between frames
            transform = self._tf_buffer.lookup_transform(
                self._parent_frame, self._child_frame, rclpy.time.Time()
            )
            self.get_logger().info(
                f"Transform between {self._parent_frame} and {self._child_frame}: \n"
                + str(transform)
            )
        except TransformException as ex:
            self.get_logger().fatal(
                f"Could not get transform between {self._parent_frame} and {self._child_frame}: {str(ex)}"
            )


class KDLFrameDemo(Node):
    """
    Class to broadcast Frames. This class consists of a static broadcaster and a dynamic broadcaster.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        # List of parts detected in the left and right bins
        # These lists contain AdvancedLogicalCameraImage objects
        self._left_bin_parts = []
        self._right_bin_parts = []

        # Flag to check if the purple pump is found
        self._found_purple_pump = False

        # Information about the parts we are looking for
        self._find_part_color = AriacPart.PURPLE
        self._find_part_type = AriacPart.PUMP

        # Camera pose in world frame
        self._right_bins_camera_pose_in_world = None
        self._left_bins_camera_pose_in_world = None
        self._part_pose_in_world = None

        # subscriber to the topic /ariac/sensors/left_bins_camera/image
        self._left_bins_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage,
            "/ariac/sensors/left_bins_camera/image",
            self.left_bins_camera_callback,
            qos_profile_sensor_data,
        )

        # subscriber to the topic /ariac/sensors/right_bins_camera/image
        self._right_bins_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage,
            "/ariac/sensors/right_bins_camera/image",
            self.right_bins_camera_callback,
            qos_profile_sensor_data,
        )

        # timer for finding the part in the left and right bins
        self._find_part_timer = self.create_timer(0.05, self.find_part_callback)

        self.get_logger().info("KDL frame demo started")

    def find_part_callback(self):
        """
        Callback function for the timer. This function is called every 0.5 seconds.
        """

        if not self._found_purple_pump:
            self.get_logger().info("Searching...")
            # Iterate over the parts detected in the left bin
            for part_pose in self._left_bin_parts:
                if (
                    part_pose.part.color == self._find_part_color
                    and part_pose.part.type == self._find_part_type
                ):  # Found purple pump
                    self._found_purple_pump = True
                    self._part_pose_in_world = self.compute_part_pose_in_world(
                        part_pose.pose, self._left_bins_camera_pose_in_world
                    )
                    break
            # Iterate over the parts detected in the right bin
            for part_pose in self._right_bin_parts:
                if (
                    part_pose.part.color == self._find_part_color
                    and part_pose.part.type == self._find_part_type
                ):  # Found purple pump
                    self._found_purple_pump = True
                    self._part_pose_in_world = self.compute_part_pose_in_world(
                        part_pose.pose, self._right_bins_camera_pose_in_world
                    )
                    break

    def compute_part_pose_in_world(
        self, part_pose_in_camera_frame, camera_pose_in_world_frame
    ):
        camera_orientation = camera_pose_in_world_frame.orientation
        camera_x = camera_pose_in_world_frame.position.x
        camera_y = camera_pose_in_world_frame.position.y
        camera_z = camera_pose_in_world_frame.position.z

        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                camera_orientation.x,
                camera_orientation.y,
                camera_orientation.z,
                camera_orientation.w,
            ),
            PyKDL.Vector(camera_x, camera_y, camera_z),
        )

        part_orientation = part_pose_in_camera_frame.orientation
        part_x = part_pose_in_camera_frame.position.x
        part_y = part_pose_in_camera_frame.position.y
        part_z = part_pose_in_camera_frame.position.z

        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                part_orientation.x,
                part_orientation.y,
                part_orientation.z,
                part_orientation.w,
            ),
            PyKDL.Vector(part_x, part_y, part_z),
        )

        frame3 = frame1 * frame2

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        rpy = rpy_from_quaternion(pose.orientation)
        output = "\n"
        output += "=" * 50 + "\n"
        output += f"Part position in world frame: \n x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z}\n"
        output += f"Part orientation in world frame: \n rpy: {rpy[0]}, {rpy[1]}, {rpy[2]}\n"
        output += "=" * 50 + "\n"
        self.get_logger().info(output)

        return pose

    def left_bins_camera_callback(self, msg: AriacAdvancedLogicalCameraImage):
        """
        Callback function for the left_bins_camera subscriber.
        """
        self._left_bin_parts.clear()
        if len(msg.part_poses) == 0:
            self.get_logger().warn("No parts detected in left bins")
            return

        if self._left_bins_camera_pose_in_world is None:
            self._left_bins_camera_pose_in_world = msg.sensor_pose

        for part_pose in msg.part_poses:
            # self.get_logger().info(
            #     f"Part detected in left bins: {part_pose.part.type} {part_pose.part.color}"
            # )
            self._left_bin_parts.append(part_pose)

    def right_bins_camera_callback(self, msg: AriacAdvancedLogicalCameraImage):
        """
        Callback function for the right_bins_camera subscriber.
        """
        self._right_bin_parts.clear()
        if len(msg.part_poses) == 0:
            self.get_logger().warn("No parts detected in right bins")
            return

        if self._right_bins_camera_pose_in_world is None:
            self._right_bins_camera_pose_in_world = msg.sensor_pose

        for part_pose in msg.part_poses:
            # self.get_logger().info(
            #     f"Part detected in right bins: {part_pose.part.type} {part_pose.part.color}"
            # )
            self._right_bin_parts.append(part_pose)


class BroadcasterDemo(Node):
    """
    Class to broadcast Frames. This class consists of a static broadcaster and a dynamic broadcaster.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        # List of parts detected in the left and right bins
        # These lists contain AdvancedLogicalCameraImage objects
        self._left_bin_parts = []
        self._right_bin_parts = []

        # Flag to check if the purple pump is found
        self._found_purple_pump = False
        # Information for broadcasting
        self._part_parent_frame = None
        self._part_frame = "purple_part"
        self._part_pose = None

        # Information about the parts we are looking for
        self._find_part_color = AriacPart.PURPLE
        self._find_part_type = AriacPart.PUMP

        # subscriber to the topic /ariac/sensors/left_bins_camera/image
        self._left_bins_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage,
            "/ariac/sensors/left_bins_camera/image",
            self.left_bins_camera_callback,
            qos_profile_sensor_data,
        )

        # subscriber to the topic /ariac/sensors/right_bins_camera/image
        self._right_bins_camera_sub = self.create_subscription(
            AriacAdvancedLogicalCameraImage,
            "/ariac/sensors/right_bins_camera/image",
            self.right_bins_camera_callback,
            qos_profile_sensor_data,
        )

        # timer for finding the part in the left and right bins
        self._find_part_timer = self.create_timer(0.05, self.find_part_callback)

        # List of transforms to be broadcast
        self._transforms = []

        # Create a dynamic broadcaster
        self._tf_dynamic_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Broadcaster demo started")

    def find_part_callback(self):
        """
        Callback function for the timer. This function is called every 0.5 seconds.
        """

        if not self._found_purple_pump:
            self.get_logger().info("Searching...")
            # Iterate over the parts detected in the left bin
            for part_pose in self._left_bin_parts:
                if (
                    part_pose.part.color == self._find_part_color
                    and part_pose.part.type == self._find_part_type
                ):  # Found purple pump
                    self._part_parent_frame = "left_bins_camera_frame"
                    self._part_pose = part_pose.pose
                    self._found_purple_pump = True
                    self.generate_transform(
                        self._part_parent_frame, self._part_frame, self._part_pose
                    )
                    break
            # Iterate over the parts detected in the right bin
            for (
                part_pose
            ) in self._right_bin_parts:  # part_pose: AdvancedLogicalCameraImage
                if (
                    part_pose.part.color == self._find_part_color
                    and part_pose.part.type == self._find_part_type
                ):  # Found purple pump
                    self._part_parent_frame = "right_bins_camera_frame"
                    self._part_pose = part_pose.pose
                    self._found_purple_pump = True
                    self.generate_transform(
                        self._part_parent_frame, self._part_frame, self._part_pose
                    )
                    break
        else:
            # Publish the transform
            self.broadcast()

    def broadcast(self):
        """
        Publish the transforms in the list of transforms to be broadcast.
        """
        self._tf_dynamic_broadcaster.sendTransform(self._transforms)

    def left_bins_camera_callback(self, msg: AriacAdvancedLogicalCameraImage):
        """
        Callback function for the left_bins_camera subscriber.
        """
        self._left_bin_parts.clear()
        if len(msg.part_poses) == 0:
            self.get_logger().warn("No parts detected in left bins")
            return

        for part_pose in msg.part_poses:
            self.get_logger().info(
                f"Part detected in left bins: {part_pose.part.type} {part_pose.part.color}"
            )
            self._left_bin_parts.append(part_pose)

    def right_bins_camera_callback(self, msg: AriacAdvancedLogicalCameraImage):
        """
        Callback function for the right_bins_camera subscriber.
        """
        self._right_bin_parts.clear()
        if len(msg.part_poses) == 0:
            self.get_logger().warn("No parts detected in right bins")
            return

        for part_pose in msg.part_poses:
            self.get_logger().info(
                f"Part detected in right bins: {part_pose.part.type} {part_pose.part.color}"
            )
            self._right_bin_parts.append(part_pose)

    def generate_transform(self, parent, child, pose):
        """
        Build a transform message and append it to the list of transforms to be broadcast.

        Args:
            parent (str): Parent frame.
            child (str): Child frame.
            pose (geometry_msgs.msg.Pose): Pose of the child frame with respect to the parent frame.

        """
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = parent
        transform_stamped.child_frame_id = child

        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z
        transform_stamped.transform.rotation.x = pose.orientation.x
        transform_stamped.transform.rotation.y = pose.orientation.y
        transform_stamped.transform.rotation.z = pose.orientation.z
        transform_stamped.transform.rotation.w = pose.orientation.w

        self._transforms.append(transform_stamped)
