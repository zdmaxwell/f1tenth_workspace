#include <rclcpp/rclcpp.hpp>
#include <IpIpoptApplication.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <limits>
#include "f1tenth_control/utils.h"
#include "f1tenth_control/mpc.h"

// Model Predictive Controller for Slash 2WD
class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("slash_mpc")
    {
        std::string home = std::getenv("HOME");
        loadCenterline(home + "/f1tenth_ws/bag_files/teleop/extracted_data/centerline_drive_data_0502_1050.csv");

        // Subscribers
        // IsaacSim-provided odometry for now without any estimation
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg)
            {
                this->poseCallback(msg);
            });

        // Publishers
        // Velocity commands from the MPC controller
        cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ackermann_cmd", 10);
        // Green markers to visualize the local waypoints
        centerline_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/mpc/centerline_points", 10);
        // Red markers to visualize the fitted path
        fit_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/mpc/polyfit", 10);

        RCLCPP_INFO(this->get_logger(), "MPC Node initialized");
    }

private:
    std::vector<double> ptsx_;
    std::vector<double> ptsy_;

    void loadCenterline(const std::string &path)
    {

        std::ifstream file(path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", path.c_str());
            return;
        }
        std::string line;
        std::getline(file, line);

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string x_str, y_str;
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str))
            {
                try
                {
                    ptsx_.push_back(std::stod(x_str));
                    ptsy_.push_back(std::stod(y_str));
                }
                catch (const std::invalid_argument &e)
                {
                    RCLCPP_WARN(this->get_logger(), "Skipping invalid row: '%s'", line.c_str());
                }
            }
        }
    }

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {   
        // Phase 1 (Get current state): Get the car's current state
        // Get x, y position states from odometry msg
        // Refers to the car's x/y position in the global frame (same as centerline)
        double px = msg->pose.pose.position.x;
        double py = msg->pose.pose.position.y;

        // Get yaw from the orientation quaternion
        // psi: which way the car is facing (yaw)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        // Convert quaternion to roll, pitch, yaw
        // pass by reference here (roll and pitch aren't used for now)
        double roll, pitch, psi;
        tf2::Matrix3x3(q).getRPY(roll, pitch, psi);

        // Current forward speed from odometry (m/s)
        double v = msg->twist.twist.linear.x;
        // Phase 1 (Get current state): Complete

        // Phase 2 (Localize/Fit the path): Find the closest waypoint and generate local waypoints
        const double max_forward_range = 30.0; // in meters
        const size_t max_points = 30;

        // Locate the closest waypoint to the current pose
        size_t closest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max(); 
        for (size_t i = 0; i < ptsx_.size(); ++i)
        {
            double dx = ptsx_[i] - px; // difference in x between waypoint and car
            double dy = ptsy_[i] - py; // difference in y between waypoint and car
            // squared distance between waypoint and car
            // note: computing squared distance instead of distance to avoid square root operation on each iteration
            double dist_sq = dx * dx + dy * dy; 
            if (dist_sq < min_dist_sq) // if current waypoint is closer than the previous closest waypoint
            {
                min_dist_sq = dist_sq; // update the closest waypoint
                closest_idx = i; // update the index of the closest waypoint
            }
        }

        std::vector<double> ptsx_local; // local waypoints in the car's body frame
        std::vector<double> ptsy_local; // local waypoints in the car's body frame
        ptsx_local.reserve(max_points);
        ptsy_local.reserve(max_points);

        size_t samples_checked = 0;
        while (ptsx_local.size() < max_points && samples_checked < ptsx_.size())
        {
            size_t idx = (closest_idx + samples_checked) % ptsx_.size(); // needed because the track is circular
            samples_checked++;

            double dx = ptsx_[idx] - px; // difference in x between waypoint and car
            double dy = ptsy_[idx] - py; // difference in y between waypoint and car

            double x_local = dx * cos(-psi) - dy * sin(-psi); // get the x position of the waypoint in the car's body frame
            double y_local = dx * sin(-psi) + dy * cos(-psi); // get the y position of the waypoint in the car's body frame

            if (x_local < 0.0 || x_local > max_forward_range)
            {
                continue;
            }

            // vectors representing the forward (x) and lateral (y) distance to waypoint[i]
            // each poseCallback call results in a new set of local waypoints for that time step
            ptsx_local.push_back(x_local);
            ptsy_local.push_back(y_local);
        }

        if (ptsx_local.size() < 6)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Insufficient forward waypoints (%zu) for polyfit", ptsx_local.size());
        }

        // y = Ac
        Eigen::VectorXd ptsx_eigen = Eigen::Map<Eigen::VectorXd>(ptsx_local.data(), ptsx_local.size()); // each row of A matrix
        Eigen::VectorXd ptsy_eigen = Eigen::Map<Eigen::VectorXd>(ptsy_local.data(), ptsy_local.size()); // y column vector
        coeffs_ = polyfit(ptsx_eigen, ptsy_eigen, 3);
        double cte = polyeval(coeffs_, 0.0);  // y error at x=0
        double epsi = -std::atan(coeffs_[1]); // heading error at x=0 (the slope of the polynomial at x=0). It will always be x=0 because the car is always at the origin in the body frame.

        // Setup state vector: [x, y, psi, v, cte, epsi]
        // cte: answers is the car to the right or left of the path
        // epsi: answers if the car's hood facing the same way as the path
        Eigen::VectorXd state(6);
        state << 0.0, 0.0, 0.0, v, cte, epsi;

        // Call the solver with the current state and coefficients
        std::vector<double> result = mpc_.Solve(state, coeffs_);
        publishVisualization(msg->header.stamp, ptsx_local, ptsy_local, coeffs_);

        v += result[1] * dt; // <- simulate updated velocity (odom_v + a0 * 0.1)

        // Publish the result
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = result[0];
        drive_msg.drive.speed = v;
        drive_msg.drive.acceleration = result[1];
        cmd_pub_->publish(drive_msg);
    }

    void publishVisualization(const rclcpp::Time &stamp,
                              const std::vector<double> &ptsx_local,
                              const std::vector<double> &ptsy_local,
                              const Eigen::VectorXd &coeffs)
    {
        if (!centerline_marker_pub_ || !fit_marker_pub_)
        {
            return;
        }

        visualization_msgs::msg::Marker centerline_marker;
        centerline_marker.header.frame_id = "base_link";
        centerline_marker.header.stamp = stamp;
        centerline_marker.ns = "mpc_centerline";
        centerline_marker.id = 0;
        centerline_marker.type = visualization_msgs::msg::Marker::POINTS;
        centerline_marker.action = visualization_msgs::msg::Marker::ADD;
        centerline_marker.scale.x = 0.07;
        centerline_marker.scale.y = 0.07;
        centerline_marker.color.g = 1.0f;
        centerline_marker.color.a = 1.0f;

        for (size_t i = 0; i < ptsx_local.size(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = ptsx_local[i];
            p.y = ptsy_local[i];
            centerline_marker.points.push_back(p);
        }

        visualization_msgs::msg::Marker fit_marker;
        fit_marker.header.frame_id = "base_link";
        fit_marker.header.stamp = stamp;
        fit_marker.ns = "mpc_polyfit";
        fit_marker.id = 0;
        fit_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        fit_marker.action = visualization_msgs::msg::Marker::ADD;
        fit_marker.scale.x = 0.05;
        fit_marker.color.r = 1.0f;
        fit_marker.color.a = 1.0f;

        const double max_x = 15.0;
        const double step = 0.3;
        for (double x = 0.0; x <= max_x; x += step)
        {
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = polyeval(coeffs, x);
            fit_marker.points.push_back(p);
        }

        centerline_marker_pub_->publish(centerline_marker);
        fit_marker_pub_->publish(fit_marker);
    }

    MPC mpc_;
    Eigen::VectorXd coeffs_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerline_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fit_marker_pub_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);
    // Construct the MPCNode and spin until shutdown
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}
