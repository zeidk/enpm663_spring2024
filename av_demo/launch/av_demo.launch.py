# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


# this function is needed
def generate_launch_description():

    ld = LaunchDescription()
    
    # Declare a command-line argument "cmd_line_parameter"
    cmd_line_parameter = DeclareLaunchArgument(
        "cmd_line_parameter",
        default_value="default_value",
        description="A parameter from the command line.",
    )

    node_params = PathJoinSubstitution(
        [FindPackageShare("av_demo"), "config", "params.yaml"]
    )

    av_actions = Node(
        package="av_demo",
        executable="av_actions_demo.py",
        remappings=[("av_lidar", "av_lidar_velodyne")],
    )

    av_sensors = Node(
        package="av_demo",
        executable="av_sensors_demo.py",
        parameters=[{'cmd_line_parameter': LaunchConfiguration('cmd_line_parameter')},node_params],
    )
    
    ld.add_action(cmd_line_parameter) 
    ld.add_action(av_actions)
    ld.add_action(av_sensors)
    return ld
