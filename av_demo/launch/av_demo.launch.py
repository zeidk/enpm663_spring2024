# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


# This function is needed
def generate_launch_description():
    ld = LaunchDescription()

    av_camera1 = Node(
        package="av_demo",
        executable="av_cameras_demo.py",
        name="av_camera1",
        remappings=[("av_camera", "av_camera1")],
    )
    av_camera2 = Node(
        package="av_demo",
        executable="av_cameras_demo.py",
        name="av_camera2",
        remappings=[("av_camera", "av_camera2")],
    )
    av_camera3 = Node(
        package="av_demo",
        executable="av_cameras_demo.py",
        name="av_camera3",
        remappings=[("av_camera", "av_camera3")],
    )
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

    # Python nodes
    av_actions_py = Node(
        package="av_demo",
        executable="av_actions_demo.py",
        remappings=[("av_lidar", "av_lidar_velodyne")],
    )
    av1 = Node(
        package="av_demo",
        executable="av_sensors_demo.py",
        name="vehicle1",
        remappings=[
            ("av_lidar", "vehicle1_lidar"),
            ("av_camera", "vehicle1_camera"),
            ("av_radar", "vehicle1_radar"),
        ],
        parameters=[node_params],
    )
    
    av2 = Node(
        package="av_demo",
        executable="av_sensors_demo.py",
        name="vehicle2",
        remappings=[
            ("av_lidar", "vehicle2_lidar"),
            ("av_camera", "vehicle2_camera"),
            ("av_radar", "vehicle2_radar"),
        ],
        parameters=[node_params],
    )
    
    av3 = Node(
        package="av_demo",
        executable="av_sensors_demo.py",
        name="vehicle3",
        remappings=[
            ("av_lidar", "vehicle3_lidar"),
            ("av_camera", "vehicle3_camera"),
            ("av_radar", "vehicle3_radar"),
        ],
        parameters=[node_params],
    )

    # C++ nodes
    av_actions_cpp = Node(
        package="av_demo",
        executable="av_actions_demo",
    )
    av_sensors_cpp = Node(
        package="av_demo",
        executable="av_sensors_demo",
        parameters=[
            {"cmd_line_parameter": LaunchConfiguration("cmd_line_parameter")},
            node_params,
        ],
    )

    ld.add_action(cmd_line_parameter)
    ld.add_action(av1)
    ld.add_action(av2)
    ld.add_action(av3)

    return ld
