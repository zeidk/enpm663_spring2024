# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node


# this function is needed
def generate_launch_description():
    ld = LaunchDescription()

    publisher = Node(package="message_test_demo", executable="message_test_exe")

    server_cpp = Node(package="message_test_demo", executable="service_server_exe")
    server_py = Node(package="message_test_demo", executable="server_exe.py")

    client_subscriber_demo = Node(
        package="message_test_demo",
        executable="service_client_subscriber_exe",
    )
    
    client_timer_demo = Node(
        package="message_test_demo",
        executable="service_client_timer_exe",
    )

    ld.add_action(publisher)
    ld.add_action(server_py)
    # ld.add_action(client_subscriber_demo)
    ld.add_action(client_timer_demo)
    return ld
