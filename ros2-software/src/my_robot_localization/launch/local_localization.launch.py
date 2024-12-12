import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), "config", "ekf.yaml")],
    )

    imu_republisher = Node(
        package="my_robot_localization",
        executable="imu_republisher.py"
    )

    desired_pose = Node(
        package="my_robot_localization",
        executable="desired_pose.py"
    )

    return LaunchDescription([
        robot_localization,
        imu_republisher,
        desired_pose,
    ])