#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class RvizRobotTrajectory(Node):
    '''
        @Brief 
        Subscribe in topic 

        Publish in the topic 
    '''

    def __init__(self):
        super().__init__("rviz_robot_trajectory")
        self.get_logger().info("Rviz robot trajectory started")

        self.rviz_robot_trajectory_msg_ = Path()
        
        self.odometry_subscriber_ = self.create_subscription(
            msg_type=Odometry,
            topic="/robot_odometry",
            callback=self.odometryCallback,
            qos_profile=10
        )
        self.rviz_robot_trajectory_publisher_ = self.create_publisher(
            msg_type=Path,
            topic="/robot/rviz/trajectory",
            qos_profile=10
        )

    def odometryCallback(self, msg):
        robot_pose_msg = PoseStamped()

        self.rviz_robot_trajectory_msg_.header.frame_id = msg.header.frame_id
        self.rviz_robot_trajectory_msg_.header.stamp = msg.header.stamp

        robot_pose_msg.header.frame_id = msg.header.frame_id
        robot_pose_msg.header.stamp = msg.header.stamp
        robot_pose_msg.pose = msg.pose.pose

        self.rviz_robot_trajectory_msg_.poses.append(robot_pose_msg)

        self.rviz_robot_trajectory_publisher_.publish(self.rviz_robot_trajectory_msg_)


def main(args=None):
    rclpy.init(args=args)
    rviz_robot_trajectory_node = RvizRobotTrajectory()
    rclpy.spin(rviz_robot_trajectory_node)
    rviz_robot_trajectory_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
