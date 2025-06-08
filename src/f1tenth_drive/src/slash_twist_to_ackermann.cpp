#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// For Traxxas Slash F1TENTH build
// Using to test teleop works with ROS2 setup
// Subscribes to a Twist message (linear and angular velocity), converts it to an AckermannDriveStamped message, and publishes it to ackermann_cmd topic
class SlashTwistToAckermann : public rclcpp::Node
{
public:
    SlashTwistToAckermann() : Node("slash_twist_to_ackermann")
    {
        loadParameters();
        logParameters();
        setupSubscribers();
        setupPublishers();
        setupTimer();

        RCLCPP_INFO(this->get_logger(), "SlashTwistToAckermann Node started");
    }

private:
    int publish_period_ms_;
    double track_width_;
    double max_speed_;
    double linear_acceleration_;
    double steering_velocity_;
    double max_steering_angle_;
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg_;
    bool is_ackermann_msg_ready_ = false; // flag to check if it's ok to publish the message, since I'm initializing ackermann_msg_ as an object and not a SharedPtr

    std::string twist_subscribe_topic_;
    std::string ackermann_publish_topic_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void setupTimer()
    {
        // create a timer based on the default or passed in publish period
        auto timer_period = std::chrono::duration<double>(publish_period_ms_ / 1000.0);
        timer_ = this->create_wall_timer(
            timer_period,
            [this]
            { timerCallback(); });
    }

    void setupPublishers()
    {
        // isaacsim will subscribe to this topic to control the vehicle
        ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            ackermann_publish_topic_, 1);
    }

    void setupSubscribers()
    {
        // this will subscribe to any node publishing Twist messages on /cmdvel and run translateTwistToAckermann() callback
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            twist_subscribe_topic_, 1,
            [this](geometry_msgs::msg::Twist::SharedPtr msg)
            {
                this->translateTwistToAckermann(msg);
            });
    }

    void loadParameters()
    {
        this->declare_parameter<int>("publish_period_ms", 20);
        this->declare_parameter<double>("track_width", 0.325); // TODO - refine value from Traxxas Slash 2WD
        this->declare_parameter<double>("max_speed", 3.0);
        this->declare_parameter<double>("linear_acceleration", 0.0);
        this->declare_parameter<double>("steering_velocity", 0.0);
        this->declare_parameter<double>("max_steering_angle", 0.523);
        this->declare_parameter<std::string>("twist_subscribe_topic", "/cmd_vel");
        this->declare_parameter<std::string>("ackermann_publish_topic", "/ackermann_cmd");

        this->get_parameter("publish_period_ms", publish_period_ms_);
        this->get_parameter("track_width", track_width_);
        this->get_parameter("max_speed", max_speed_);
        this->get_parameter("linear_acceleration", linear_acceleration_);
        this->get_parameter("steering_velocity", steering_velocity_);
        this->get_parameter("max_steering_angle", max_steering_angle_);
        this->get_parameter("twist_subscribe_topic", twist_subscribe_topic_);
        this->get_parameter("ackermann_publish_topic", ackermann_publish_topic_);
    }

    void logParameters()
    {
        RCLCPP_INFO(this->get_logger(), "===== SlashTwistToAckermann Parameters =====");
        RCLCPP_INFO(this->get_logger(), "Twist subscribe topic: %s", twist_subscribe_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Ackermann publish topic: %s", ackermann_publish_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Track Width: %.3f m", track_width_);
        RCLCPP_INFO(this->get_logger(), "Max speed: %.3f m/s", max_speed_);
        RCLCPP_INFO(this->get_logger(), "Linear acceleration: %.3f m/s^2", linear_acceleration_);
        RCLCPP_INFO(this->get_logger(), "Steering velocity: %.3f rad/s", steering_velocity_);
        RCLCPP_INFO(this->get_logger(), "Max steering angle: %.3f rad", max_steering_angle_);
        RCLCPP_INFO(this->get_logger(), "Publish period: %d ms", publish_period_ms_);
        RCLCPP_INFO(this->get_logger(), "=========================================");
    }

    double clamp(double value, double min_value, double max_value)
    {
        return std::min(std::max(value, min_value), max_value);
    }

    // compute the proper steering angle given the car's current speed and angular speed using Ackermann steering
    double calculateAckermannSteeringAngle(const double speed, const double angular_speed)
    {
        if (std::abs(speed) < 1e-5)
        {
            RCLCPP_WARN(this->get_logger(), "Speed near zero: (v=%.5f). Commanding steering angle to 0.", speed);
            return 0.0;
        }
        return std::atan(track_width_ * angular_speed / speed);
    }

    // callback to translate Twist messages to AckermannDriveStamped
    void translateTwistToAckermann(const geometry_msgs::msg::Twist::SharedPtr &msg)
    {
        // create the Ackermann message to publish
        ackermann_msg_.header.stamp = this->now();
        ackermann_msg_.drive.speed = clamp(msg->linear.x, -max_speed_, max_speed_);
        double steering_angle = calculateAckermannSteeringAngle(
            ackermann_msg_.drive.speed, msg->angular.z);
        ackermann_msg_.drive.steering_angle = clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
        ackermann_msg_.drive.acceleration = linear_acceleration_;
        ackermann_msg_.drive.steering_angle_velocity = steering_velocity_;

        is_ackermann_msg_ready_ = true;
    }

    void timerCallback()
    {
        if (is_ackermann_msg_ready_)
        {
            ackermann_pub_->publish(ackermann_msg_);
            is_ackermann_msg_ready_ = false;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlashTwistToAckermann>());
    rclcpp::shutdown();
    return 0;
}