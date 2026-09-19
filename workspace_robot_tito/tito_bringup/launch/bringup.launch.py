from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory("mi_robot")

    urdf_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")

    robot_description = Command(["xacro ", urdf_file])

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                os.path.join(pkg_path, "config", "controllers.yaml")
            ],
            output="screen"
        )
    ])
