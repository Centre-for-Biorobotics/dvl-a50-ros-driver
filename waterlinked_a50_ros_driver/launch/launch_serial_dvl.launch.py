from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    dvl_node = Node(
        package="waterlinked_a50_ros_driver",
        executable="dvl_serial_publisher",
        name="a50_dlv_serial",
        output="screen",
        parameters=[
            {'serial_port': '/dev/ttyUSB0'}
        ]
    )

    return LaunchDescription([
        dvl_node
    ])