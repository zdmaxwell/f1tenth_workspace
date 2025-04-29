from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("publish_period_ms", default_value="20", description="period to publish at (ms)"),
        DeclareLaunchArgument("track_width", default_value="0.325", description="distance between the wheels on the front axle (m)"),
        DeclareLaunchArgument("max_speed", default_value="3.0", description="max speed the car can reach (m/s)"),
        DeclareLaunchArgument("linear_acceleration", default_value="0.0", description="linear acceleration (m/s^2)"),
        DeclareLaunchArgument("max_steering_angle", default_value="0.523", description="max allowable steering angle (rads)"),
        DeclareLaunchArgument("steering_velocity", default_value="0.0", description="how quickly the steering angle changes (radians/s)"),
        DeclareLaunchArgument("twist_subscribe_topic", default_value="/cmd_vel", description="topic to subscribe to that uses Twist msgs"),
        DeclareLaunchArgument("ackermann_publish_topic", default_value="/ackermann_cmd", description="topic to publish to that uses Ackermann msgs"),
        
        Node(
            package='f1tenth_drive',
            executable='slash_twist_to_ackermann',
            name='slash_twist_to_ackermann',
            output="screen",
            parameters=[{
                'publish_period_ms': LaunchConfiguration('publish_period_ms'),
                'track_width': LaunchConfiguration('track_width'),
                'max_speed': LaunchConfiguration('max_speed'),
                'linear_acceleration': LaunchConfiguration('linear_acceleration'),
                'max_steering_angle': LaunchConfiguration('max_steering_angle'),
                'steering_velocity': LaunchConfiguration('steering_velocity'),
                'twist_subscribe_topic': LaunchConfiguration('twist_subscribe_topic'),
                'ackermann_publish_topic': LaunchConfiguration('ackermann_publish_topic'),
            }]
        ),
    ])