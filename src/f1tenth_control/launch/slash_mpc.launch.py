from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_parameters = PathJoinSubstitution(
        [FindPackageShare("f1tenth_control"), "config", "slash_mpc.yaml"]
    )
    default_centerline = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "f1tenth_ws",
            "bag_files",
            "teleop",
            "extracted_data",
            "centerline_drive_data_0502_1050.csv",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_parameters),
            DeclareLaunchArgument("centerline_csv", default_value=default_centerline),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("ackermann_topic", default_value="/ackermann_cmd"),
            Node(
                package="f1tenth_control",
                executable="slash_mpc",
                name="slash_mpc",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"centerline_csv": LaunchConfiguration("centerline_csv")},
                ],
                remappings=[
                    ("odom", LaunchConfiguration("odom_topic")),
                    ("ackermann_cmd", LaunchConfiguration("ackermann_topic")),
                ],
            ),
        ]
    )
