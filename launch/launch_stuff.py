from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
        ),
        Node(
            package="teleop",
            executable="joyBroadcast",
            name="joyBroadcast",
        ),
        Node(
            package="spicontrol",
            executable="spicontrol",
            name="spicontrol",
        ),
    ])
