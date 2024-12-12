#!/usr/bin/env python3

import math
import numpy as np
from shapely.geometry import Point
from rtree import index
import uuid
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rcl_interfaces.msg import SetParametersResult

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Range, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from tf_transformations import euler_from_quaternion


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.get_logger().info(f"{'='*20}\t Robot controller started \t{'='*20}")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.195)
        self.declare_parameter("max_goal_error", 0.1)
        self.declare_parameter("real_robot", 0)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.max_goal_error_ = self.get_parameter("max_goal_error").get_parameter_value().double_value
        self.real_robot_ = self.get_parameter("real_robot").get_parameter_value().integer_value

        self.get_logger().info(f"Real robot = {self.real_robot_}")

        self.kp_ = 0.25
        self.kv_ = 6 / math.pi
        self.robot_position_x_ = 0.0
        self.robot_position_y_ = 0.0
        self.robot_theta_angle_ = 0.0
        self.robot_end_goal_position_x_ = None
        self.robot_end_goal_position_y_ = None
        self.enable_controller_ = False
        self.robot_goal_destination_ = False
        self.eplison_attracness_ = 5E-1
        self.eplison_repulsiveness_ = 10E-3
        self.objects_around_robot_radius_ = 0.3
        self.distance_between_each_obstacles_ = 0.1
        self.rtree_index_ = index.Index()
        self.lock_ = threading.Lock()

        self.rviz2_robot_trajectory_ = Path()

        self.converte_wheel_speed_to_robot_speed_ = np.array(
            [
                [self.wheel_radius_/2, self.wheel_radius_/2], 
                [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]
            ]
        )

        self.robot_localization_subscriber_ = self.create_subscription(
            msg_type=Odometry,
            #topic="/odometry/filtered",
            topic="/robot_odometry",
            callback=self.robotLocalizationCallback,
            qos_profile=10
        )
        self.robot_goal_subscriber_ = self.create_subscription(
            msg_type=Pose,
            topic="/robot/desired_pose",
            callback=self.robotEndGoalCallback,
            qos_profile=10
        )
        self.ultrasonic_sensor_middle_subscriber = self.create_subscription(
            #Range,
            #"/robot/sensor/ultrasonic/center",
            PointCloud2,
            "/ultrasonic_sensor_1",
            self.create_ultrasonic_callback("/ultrasonic_sensor_1"),
            10
            #QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.ultrasonic_sensor_left_subscriber = self.create_subscription(
            #Range,
            #"/robot/sensor/ultrasonic/left",
            PointCloud2,
            "/ultrasonic_sensor_2",
            self.create_ultrasonic_callback("/ultrasonic_sensor_2"),
            10
            #QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.ultrasonic_sensor_right_subscriber = self.create_subscription(
            #Range,
            #"/robot/sensor/ultrasonic/right",
            PointCloud2,
            "/ultrasonic_sensor_3",
            self.create_ultrasonic_callback("/ultrasonic_sensor_3"),
            10
            #QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.wheel_speed_commands_publisher_ = self.create_publisher(
            msg_type=Float64MultiArray,
            #topic="/robot/control/wheel/speed",                 # real robot 
            topic="/simple_velocity_controller/commands",      # controller option 1 
            #msg_type=TwistStamped,                             # controller option 2
            #topic="/robot_controller/cmd_vel",                 # controller option 2 
            qos_profile=10
        )
        self.create_timer(
            timer_period_sec=5E-3,
            callback=self.robotController
        )
        self.create_timer(
            timer_period_sec=5000,
            callback=self.forgetObstacle
        )
        self.rviz2_robot_trajectory_publisher_ = self.create_publisher(
            msg_type=Path,
            topic="/rviz2/robot/trajectory",
            qos_profile=10
        )
        self.rviz2_obstacle_publisher_ = self.create_publisher(
            msg_type=MarkerArray,
            topic="/rviz2/robot/obstacles",
            qos_profile=10
        )
        
        self.debug_ = self.create_publisher(
            msg_type=Float64MultiArray,
            topic="/debug",
            qos_profile=10
        )

        self.declare_parameter("no_ultrassonic", 1)
        self.no_ultrassonic_ = self.get_parameter("no_ultrassonic").get_parameter_value().integer_value
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == "no_ultrassonic":
                self.no_ultrassonic_ = 0
        return SetParametersResult(successful=True)

    def forgetObstacle(self) -> None:
        self.rtree_index_ = index.Index()

    def getNearbyPoints_(self, point: Point) -> list:
        '''
            Gets a list of all abstacle points near the robot
        '''
        d = self.objects_around_robot_radius_
        with self.lock_:
            return list(self.rtree_index_.intersection(
                (point.x - d, point.y - d, point.x + d, point.y + d),
                objects=True
                ))

    def checkNearbyPoint_(self, point: Point) -> bool:
        '''
            Checks if there is a point nearby, if so it return false, otherwise it returns true 
        '''
        d = self.distance_between_each_obstacles_ 
        with self.lock_:
            nearby_points = list(self.rtree_index_.intersection(
                (point.x - d, point.y - d, point.x + d, point.y + d),
                objects=True
                ))
        if nearby_points == []:
            return True
        return False
    
    def create_ultrasonic_callback(self, topic_name):
        def ultrasonicCallback(msg):
            '''
                Callback to calculate the x and y coordenates of the obstacle using the middle ultrasonic sensor
            '''
            if self.real_robot_ == 0:
                points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                min_distance = float("inf")
                for point in points:
                    x, y, z = point
                    distance = math.sqrt(x**2 + y**2 + z**2)  
                    if distance < min_distance:
                        min_distance = distance

                if min_distance == float("inf"):
                    return
            else:
                if msg.range <= 0:
                    return
                if msg.range >= 60.0:
                    return
                min_distance = msg.range / 100.0

            if topic_name == "/ultrasonic_sensor_1":
                # 0 deg
                angle = self.robot_theta_angle_
            elif topic_name == "/ultrasonic_sensor_2":
                # +30 deg
                angle = self.robot_theta_angle_ + math.pi / 6
            elif topic_name == "/ultrasonic_sensor_3":
                # -30 deg
                angle = self.robot_theta_angle_ - math.pi / 6

            obstacle_point = Point(
                self.robot_position_x_ + min_distance * math.cos(angle),
                self.robot_position_y_ + min_distance * math.sin(angle)
            )
            
            if self.no_ultrassonic_ == 0:
                if self.checkNearbyPoint_(obstacle_point):
                    with self.lock_:
                        self.rtree_index_.insert(
                            uuid.uuid1().int>>64, 
                            (obstacle_point.x, obstacle_point.y, obstacle_point.x, obstacle_point.y), 
                            obj=obstacle_point
                        )
                             
                nearby_obstacles = self.getNearbyPoints_(Point(self.robot_position_x_, self.robot_position_y_))
                self.rviz2ObstaclePublisher_(nearby_obstacles)       

        return ultrasonicCallback

    def robotLocalizationCallback(self, msg):
        '''
            Callback for the robot localization
        '''
        self.robot_position_x_ = msg.pose.pose.position.x
        self.robot_position_y_ = msg.pose.pose.position.y
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, self.robot_theta_angle_ = euler_from_quaternion(q)
        
        self.rviz2PathPublisher_(msg)
    
    def robotEndGoalCallback(self, msg):
        '''
            Callback for the robot end goal
        '''
        self.robot_end_goal_position_x_ = msg.position.x
        self.robot_end_goal_position_y_ = msg.position.y
        self.enable_controller_ = True
        self.robot_goal_destination_ = False

    def robotController(self):
        '''
            This function applies the linear and angular speed necessary for the robot to follow the desired path
        '''
        if( self.enable_controller_ == True ):
            dx, dy = self.calcAPF_()
            desired_angle = math.atan2(dy, dx)
            self.vectorialController_(dx, dy)

    def vectorialController_(self, dx: float, dy: float) -> None:
        apf_vector = np.array([dx, dy])
        robot_orientation_vector = np.array([math.cos(self.robot_theta_angle_), math.sin(self.robot_theta_angle_)])

        # Produto escalar
        dot_product = np.dot(apf_vector, robot_orientation_vector)
        apf_magnitude = np.linalg.norm(apf_vector)
        robot_orientation_magnitude = np.linalg.norm(robot_orientation_vector)
        vectors_angle = math.acos(dot_product / (apf_magnitude * robot_orientation_magnitude))
        # Produto vetorial
        cross_product = np.cross(robot_orientation_vector, apf_vector)

        if(cross_product > 0):
            desired_angle = vectors_angle + self.robot_theta_angle_
        elif(cross_product < 0):
            desired_angle = self.robot_theta_angle_ - vectors_angle
        elif(cross_product == 0):
            desired_angle = 0

        robot_distance_error = self.goalDistanceError_()
        robot_angle_error = self.angleError_(desired_angle)

        desired_robot_linear_speed = self.kp_ * robot_distance_error
        desired_robot_angular_speed = self.kv_ * robot_angle_error

        desired_robot_linear_speed = self.limitSpeed_(
                value=desired_robot_linear_speed,
                max=0.15,
                min=-0.15
            )
        desired_robot_angular_speed = self.limitSpeed_(
                value=desired_robot_angular_speed,
                max=0.5,
                min=-0.5
            )
        self.publishSpeed_(
            desired_robot_linear_speed=desired_robot_linear_speed,
            desired_robot_angular_speed=desired_robot_angular_speed,
            robot_distance_error=robot_distance_error
            )

    def publishSpeed_(self, desired_robot_linear_speed: float, desired_robot_angular_speed: float, robot_distance_error: float) -> None:
        wheel_speed_msg_ = Float64MultiArray()      # controller option 1 
        #wheel_speed_msg_ = TwistStamped()          # controller option 2
        robot_velocity = np.array(
                [
                    [desired_robot_linear_speed],
                    [desired_robot_angular_speed]
                ]
            )
        desired_wheel_speed = np.matmul(np.linalg.inv(self.converte_wheel_speed_to_robot_speed_), robot_velocity)
        
        #wheel_speed_msg_.data = [desired_wheel_speed[0,0], desired_wheel_speed[1,0]] # real robot
        wheel_speed_msg_.data = [desired_wheel_speed[1,0], desired_wheel_speed[0,0]] # Left and Right wheel order controller 1
        #wheel_speed_msg_.twist.linear.x = desired_robot_linear_speed        # controller option 2 
        #wheel_speed_msg_.twist.angular.z = desired_robot_angular_speed      # controller option 2
        if( robot_distance_error < self.max_goal_error_ ):
            self.robot_goal_destination_ = True
            wheel_speed_msg_.data = [0.0, 0.0]
            #wheel_speed_msg_.twist.linear.x = 0.0          # controller option 2
            #wheel_speed_msg_.twist.angular.z = 0.0         # controller option 2 
        self.wheel_speed_commands_publisher_.publish(wheel_speed_msg_)
        self.debug_.publish(wheel_speed_msg_)

    def calcAPF_(self) -> list:
        eplison_attracness = self.eplison_attracness_ 
        eplison_repulsiveness = self.eplison_repulsiveness_ 
        R = self.objects_around_robot_radius_ 
        x, y = self.robot_position_x_, self.robot_position_y_
        a, b = self.robot_end_goal_position_x_, self.robot_end_goal_position_y_
        
        dx = -eplison_attracness * (x - a)
        dy = -eplison_attracness * (y - b)

        #self.rviz2ObstaclePublisher_(nearby_obstacles)

        if self.no_ultrassonic_ == 0: 
            nearby_obstacles = self.getNearbyPoints_(Point(x, y))
            for obstacle_obj in nearby_obstacles:
                obstacle_point = obstacle_obj.object
                obstacle_distance = math.sqrt( (x - obstacle_point.x)**2 + (y - obstacle_point.y)**2 )
                num = -eplison_repulsiveness * (R - obstacle_distance)
                den = R * obstacle_distance**4
                dx -= num * (x - obstacle_point.x) / den
                dy -= num * (y - obstacle_point.y) / den
            
        return dx, dy
    
    def angleError_(self, desired_angle: float) -> float:
        return desired_angle - self.robot_theta_angle_

    def goalDistanceError_(self) -> float:
        x, y = self.robot_position_x_, self.robot_position_y_
        a, b = self.robot_end_goal_position_x_, self.robot_end_goal_position_y_
        return math.sqrt((x-a)**2 + (y-b)**2)
        
    def limitSpeed_(self, value:float, max:float, min:float) -> float:
        if ( (value > max) ):
            return max
        if ( (value < min) ):
            return min
        return value
    
    def rviz2ObstaclePublisher_(self, obstacles: list) -> None:
        marker_array = MarkerArray()
        for idx, obstacle_obj in enumerate(obstacles):
            obstacle_point = obstacle_obj.object 

            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle_point.x
            marker.pose.position.y = obstacle_point.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.rviz2_obstacle_publisher_.publish(marker_array)

    def rviz2PathPublisher_(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = msg.header.stamp
        pose.pose = msg.pose.pose

        self.rviz2_robot_trajectory_.header.frame_id = msg.header.frame_id
        self.rviz2_robot_trajectory_.poses.append(pose)

        self.rviz2_robot_trajectory_publisher_.publish(self.rviz2_robot_trajectory_)


def main(args=None):
    rclpy.init(args=args)
    robot_controller_node = RobotController()
    rclpy.spin(robot_controller_node)
    robot_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
