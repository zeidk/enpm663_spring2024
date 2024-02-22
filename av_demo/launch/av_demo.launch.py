# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node

# import os
# from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


# this function is needed
def generate_launch_description():
    ld = LaunchDescription()

    # Declare a command-line argument "cmd_line_parameter"
    cmd_line_parameter = DeclareLaunchArgument(
        "cmd_line_parameter",
        default_value="default_value",
        description="A parameter from the command line.",
    )

    # Path to the parameters file
    node_params = PathJoinSubstitution(
        [FindPackageShare("av_demo"), "config", "params.yaml"]
    )

    av_actions = Node(
        package="av_demo",
        executable="av_actions_demo.py",
    )
    av_sensors = Node(
        package="av_demo",
        executable="av_sensors_demo.py",
        parameters=[
            {"cmd_line_parameter": LaunchConfiguration("cmd_line_parameter")},
            node_params,
        ],
    )

    ld.add_action(cmd_line_parameter)
    # ld.add_action(av_camera1)
    # ld.add_action(av_camera2)
    # ld.add_action(av_camera3)
    ld.add_action(av_sensors)
    return ld
