from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("spicontrol"),
        "config",
        "spicontrol_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="spicontrol",
            executable="spicontrol",
            name="spicontrol",
            # motor_usb_port / rpm_usb_port live in config/spicontrol_params.yaml
            parameters=[config],
        )
    ])