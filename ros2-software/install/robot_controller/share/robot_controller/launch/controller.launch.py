from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(
        name="wheel_radius",
        default_value="0.033"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        name="wheel_separation",
        default_value="0.17"
    )
    wheel_radius_error_arg = DeclareLaunchArgument(
        name="wheel_radius_error",
        default_value="0.038"
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        name="wheel_separation_error",
        default_value="0.195"
    )
    real_robot_arg = DeclareLaunchArgument(
        name="real_robot",
        default_value="0"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    wheel_radius_error = LaunchConfiguration("wheel_radius_error")
    wheel_separation_error = LaunchConfiguration("wheel_separation_error")
    real_robot = LaunchConfiguration("real_robot")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller", # option 1 for controller
            #"robot_controller", # option 2 for controller
            "--controller-manager",
            "/controller_manager"
        ]
    )

    velocity_controller = Node(
        package="robot_controller",
        executable="velocity_controller.py",
        parameters=[
            {
                "wheel_radius": wheel_radius,
                "wheel_separation": wheel_separation
            }
        ]
    )
    
    measure_speed = Node(
        package="robot_controller",
        executable="measure_speed.py",
        parameters=[
            {
                "wheel_radius": wheel_radius,
                "wheel_separation": wheel_separation_error,
                "real_robot": real_robot
            }
        ]
    )

    measure_speed_noisy = Node(
        package="robot_controller",
        executable="measure_speed_noisy.py",
        parameters=[
            {
                "wheel_radius": wheel_radius_error,
                "wheel_separation": wheel_separation_error
            }
        ]
    )
    
    wheel_odometry = Node(
        package="robot_controller",
        executable="wheel_odometry.py"
    )

    wheel_odometry_noisy = Node(
        package="robot_controller",
        executable="wheel_odometry_noisy.py"
    )

    return LaunchDescription(
        [
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            real_robot_arg,
            joint_state_broadcaster_spawner,
            controller_manager,
            velocity_controller,
            measure_speed,
            wheel_odometry,
            measure_speed_noisy,
            wheel_odometry_noisy,
        ]
    )
