#!/usr/bin/env python3 

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped


class SpeedController(Node):
    '''
        @Brief 
        Subscribe in topic "/target_robot_speed/commands" and recives the desired linear 
        and angular velocities for the locomotion of the robot.
        Uses interface "TwistStamped", from the "geometry_msgs" library, to recive data.

        Publish in the topic "/simple_velocity_controller/commands" the linear and angular 
        velocities for the Gazebo controller manager. 
        Uses interface "Float64MultiArray", from the "std_msgs" library, to send data.
    '''
    
    def __init__(self):
        super().__init__("speed_controller")
        self.get_logger().info(f"{'='*20}\t Speed controller initialized  \t{'='*20}")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.get_logger().info(f"Wheel radius = {self.wheel_radius_}")
        self.get_logger().info(f"Wheel separation = {self.wheel_separation_}")

        self.converte_wheel_speed_to_robot_speed_ = np.array(
            [
                [self.wheel_radius_/2, self.wheel_radius_/2], 
                [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]
            ]
        )

        self.velocity_subscriber_ = self.create_subscription(
            msg_type=TwistStamped,
            topic="/target_robot_speed/commands",
            callback=self.wheelSpeedCallback,
            qos_profile=10
        )
        self.wheel_speed_commands_publisher = self.create_publisher(
            msg_type=Float64MultiArray,
            topic="/simple_velocity_controller/commands",
            qos_profile=10
        )

    def wheelSpeedCallback(self, msg):
        wheel_speed_msg = Float64MultiArray()
        robot_velocity = np.array(
            [
                [msg.twist.linear.x],
                [msg.twist.angular.z]
            ]
        )
        wheel_speed = np.matmul(np.linalg.inv(self.converte_wheel_speed_to_robot_speed_), robot_velocity)
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]] # Left and Right wheel order
        
        self.wheel_speed_commands_publisher.publish(wheel_speed_msg)


def main(args=None):
    rclpy.init(args=args)
    velocity_controller_node = SpeedController()
    rclpy.spin(velocity_controller_node)
    velocity_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    