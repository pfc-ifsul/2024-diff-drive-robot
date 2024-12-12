#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose


def main(args=None):
    rclpy.init(args=args)
    desired_pose_node = Node("desired_pose")
    time.sleep(1)
    desired_pose_publisher_ = desired_pose_node.create_publisher(
        msg_type=Pose,
        topic="/robot/desired_pose",
        qos_profile=10
    )
    rclpy.spin(desired_pose_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
