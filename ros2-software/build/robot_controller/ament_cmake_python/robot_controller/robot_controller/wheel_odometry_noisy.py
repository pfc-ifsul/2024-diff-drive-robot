#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, TwistStamped

from tf2_ros import TransformBroadcaster
from tf_transformations  import quaternion_from_euler


class WheelOdometry(Node):
    '''
        @Brief
        Subscribe in the topic "/measured_robot_speed"

        Publish in the topic "/robot_odometry"
    '''

    def __init__(self):
        super().__init__("wheel_odometry_noisy")
        self.get_logger().info(f"{'='*20}\t Wheel odometry noisy started \t{'='*20}")

        self.declare_parameter("wheel_radius", 0.038)
        self.declare_parameter("wheel_separation", 0.19)
        self.declare_parameter("real_robot", 0)
        
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.real_robot_ = self.get_parameter("real_robot").get_parameter_value().integer_value
        
        self.get_logger().info(f"Wheel radius = {self.wheel_radius_}")
        self.get_logger().info(f"Wheel separation = {self.wheel_separation_}")

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.odometry_msg_ = Odometry()
        self.odometry_msg_.header.frame_id = "odom"
        self.odometry_msg_.child_frame_id = "base_footprint_ekf"
        self.odometry_msg_.pose.pose.orientation.x = 0.0
        self.odometry_msg_.pose.pose.orientation.y = 0.0
        self.odometry_msg_.pose.pose.orientation.z = 0.0
        self.odometry_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        if self.real_robot_ == 0:
            self.robot_speed_subscriber_ = self.create_subscription(
                msg_type=Float64MultiArray,                # controller option 1 
                topic="/measured_robot_speed_noisy",       # controller option 1 
                #msg_type=TwistStamped,                    # controller option 2
                #topic="/robot_controller/cmd_vel_out",    # controller option 2
                callback=self.robotSpeedCallback,
                qos_profile=10
            )
        else:
            
        
        self.odometry_publisher_ = self.create_publisher(
            msg_type=Odometry,
            topic="/robot_odometry_noisy",
            qos_profile=10
        )

        #self.prev_time_ = self.get_clock().now()       # controller option 2

    def robotSpeedCallback(self, msg):
        
        linear_vel = msg.data[0]                # controller option 1 
        angular_vel = msg.data[1]               # controller option 1 
        dt = msg.data[2]                        # controller option 1 

        '''
        linear_vel = msg.twist.linear.x                             # controller option 2
        angular_vel = msg.twist.angular.z                           # controller option 2
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_      # controller option 2
        dt = dt.nanoseconds / S_TO_NS                               # controller option 2
        self.prev_time_ = Time.from_msg(msg.header.stamp)          # controller option 2
        '''

        self.theta_ += angular_vel*dt + np.random.normal(0.0, 0.00005)
        self.x_ += linear_vel*dt*math.cos(self.theta_) + np.random.normal(0.0, 0.00005)
        self.y_ += linear_vel*dt*math.sin(self.theta_) + np.random.normal(0.0, 0.00005)

        
        quaternion = quaternion_from_euler(0, 0, self.theta_)
        self.odometry_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odometry_msg_.pose.pose.orientation.x = quaternion[0]
        self.odometry_msg_.pose.pose.orientation.y = quaternion[1]
        self.odometry_msg_.pose.pose.orientation.z = quaternion[2]
        self.odometry_msg_.pose.pose.orientation.w = quaternion[3]
        self.odometry_msg_.pose.pose.position.x = self.x_
        self.odometry_msg_.pose.pose.position.y = self.y_
        self.odometry_msg_.pose.pose.position.z = 0.0
        self.odometry_msg_.twist.twist.linear.x = linear_vel
        self.odometry_msg_.twist.twist.linear.y = 0.0
        self.odometry_msg_.twist.twist.linear.z = 0.0
        self.odometry_msg_.twist.twist.angular.x = 0.0
        self.odometry_msg_.twist.twist.angular.y = 0.0
        self.odometry_msg_.twist.twist.angular.z = angular_vel

        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation.x = quaternion[0]
        self.transform_stamped_.transform.rotation.y = quaternion[1]
        self.transform_stamped_.transform.rotation.z = quaternion[2]
        self.transform_stamped_.transform.rotation.w = quaternion[3]

        self.odometry_publisher_.publish(self.odometry_msg_)
        self.br_.sendTransform(self.transform_stamped_)

        

def main(args=None):
    rclpy.init(args=args)
    wheel_odometry_node = WheelOdometry()
    rclpy.spin(wheel_odometry_node)
    wheel_odometry_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
