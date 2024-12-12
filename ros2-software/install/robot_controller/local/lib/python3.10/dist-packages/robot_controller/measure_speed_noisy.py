#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class MeasureSpeed(Node):
    '''
        @Brief 
        Subscribe in topic ""

        Publishe in topic ""
    '''

    def __init__(self):
        super().__init__("measure_speed_noisy")
        self.get_logger().info(f"{'='*20}\t Measure robot speed noisy started \t{'='*20}")

        self.declare_parameter("wheel_radius", 0.038)
        self.declare_parameter("wheel_separation", 0.19)
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.get_logger().info(f"Wheel radius = {self.wheel_radius_}")
        self.get_logger().info(f"Wheel separation = {self.wheel_separation_}")

        self.prev_left_wheel_theta_ = 0.0
        self.prev_right_wheel_theta_ = 0.0
        self.prev_time_ = self.get_clock().now()

        self.converte_wheel_speed_to_robot_speed_ = np.array(
            [
                [self.wheel_radius_/2, self.wheel_radius_/2], 
                [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]
            ]
        )

        self.joint_state_subscriber_ = self.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            callback=self.jointCallback,
            qos_profile=10
        )
        self.calculated_robot_speed_publish_ = self.create_publisher(
            msg_type=Float64MultiArray,
            topic="/measured_robot_speed_noisy",
            qos_profile=10
        )
        self.calculated_wheel_speed_publish_ = self.create_publisher(
            msg_type=Float64MultiArray,
            topic="/measured_wheel_speed_noisy",
            qos_profile=10
        )

    def jointCallback(self, msg):
        robot_speed_msg = Float64MultiArray()
        wheel_speed_msg = Float64MultiArray()

        d_theta_left = msg.position[1] - self.prev_left_wheel_theta_ + np.random.normal(0.0, 0.005)
        d_theta_right = msg.position[0] - self.prev_right_wheel_theta_ + np.random.normal(0.0, 0.005)
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_ 
        dt = dt.nanoseconds / S_TO_NS

        omega_l = d_theta_left / dt
        omega_r = d_theta_right / dt
        wheel_velocity = np.array(
            [
                [omega_r],
                [omega_l]
            ]
        )
        robot_speed = np.matmul(self.converte_wheel_speed_to_robot_speed_, wheel_velocity)

        robot_speed_msg.data = [robot_speed[0,0], robot_speed[1,0], dt] # Linear and Angular velocity order and last the time used to calculate
        self.calculated_robot_speed_publish_.publish(robot_speed_msg)
        wheel_speed_msg.data = [omega_l, omega_r] # Wheel (left and right) Angular velocity
        self.calculated_wheel_speed_publish_.publish(wheel_speed_msg)

        self.prev_left_wheel_theta_ = msg.position[1]
        self.prev_right_wheel_theta_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)


def main(args=None):
    rclpy.init(args=args)
    angular_velocity_encoder_node = MeasureSpeed()
    rclpy.spin(angular_velocity_encoder_node)
    angular_velocity_encoder_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
