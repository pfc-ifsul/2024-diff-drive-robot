from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    robot_controller = Node(
        package="robot_decision_making",
        executable="robot_controller.py",
        parameters=[{"real_robot": 0}],
    )

    return LaunchDescription(
        [
            robot_controller,
        ]
    )
