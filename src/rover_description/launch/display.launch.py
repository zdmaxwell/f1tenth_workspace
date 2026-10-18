from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rover_description")
    model_file = PathJoinSubstitution([package_share, "urdf", "rover.urdf.xacro"])
    params_file = PathJoinSubstitution([package_share, "config", "model_parameters.yaml"])
    rviz_file = PathJoinSubstitution([package_share, "rviz", "rover.rviz"])

    payload_mass = LaunchConfiguration("payload_mass_kg")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            model_file,
            " params_file:=",
            params_file,
            " payload_mass_kg:=",
            payload_mass,
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "payload_mass_kg",
                default_value="0.0",
                description="Payload mass fixed to the wagon body for this model.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the Rover model configuration.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_file],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
