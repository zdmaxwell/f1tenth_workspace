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

        // Subscribing to IsaacSim-provided odometry for now without any estimation
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg)
            {
                this->poseCallback(msg);
            });

        // Publish velocity commands from the MPC controller
        cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ackermann_cmd", 10);
        centerline_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/mpc/centerline_points", 10);
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
        // Get x, y position states from odometry msg
        double px = msg->pose.pose.position.x;
        double py = msg->pose.pose.position.y;

        // Get yaw from the orientation quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, psi;
        tf2::Matrix3x3(q).getRPY(roll, pitch, psi);

        // TODO - MPC Solver goes below to generate control output
        double v = msg->twist.twist.linear.x;

        const double max_forward_range = 30.0;
        const size_t max_points = 30;

        // locate closest waypoint to current pose
        size_t closest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < ptsx_.size(); ++i)
        {
            double dx = ptsx_[i] - px;
            double dy = ptsy_[i] - py;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }

        std::vector<double> ptsx_local;
        std::vector<double> ptsy_local;
        ptsx_local.reserve(max_points);
        ptsy_local.reserve(max_points);

        size_t samples_checked = 0;
        while (ptsx_local.size() < max_points && samples_checked < ptsx_.size())
        {
            size_t idx = (closest_idx + samples_checked) % ptsx_.size();
            samples_checked++;

            double dx = ptsx_[idx] - px;
            double dy = ptsy_[idx] - py;

            double x_local = dx * cos(-psi) - dy * sin(-psi); // forward component
            double y_local = dx * sin(-psi) + dy * cos(-psi); // lateral component

            if (x_local < 0.0 || x_local > max_forward_range)
            {
                continue;
            }

            ptsx_local.push_back(x_local);
            ptsy_local.push_back(y_local);
        }

        if (ptsx_local.size() < 6)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Insufficient forward waypoints (%zu) for polyfit", ptsx_local.size());
        }

        Eigen::VectorXd ptsx_eigen = Eigen::Map<Eigen::VectorXd>(ptsx_local.data(), ptsx_local.size());
        Eigen::VectorXd ptsy_eigen = Eigen::Map<Eigen::VectorXd>(ptsy_local.data(), ptsy_local.size());
        coeffs_ = polyfit(ptsx_eigen, ptsy_eigen, 3);
        double cte = polyeval(coeffs_, 0.0);  // y error at x=0
        double epsi = -std::atan(coeffs_[1]); // heading error at x=0

        // Setup state vector: [x, y, psi, v, cte, epsi]
        Eigen::VectorXd state(6);
        state << 0.0, 0.0, 0.0, v, cte, epsi;

        // Call the solver
        std::vector<double> result = mpc_.Solve(state, coeffs_);
        publishVisualization(msg->header.stamp, ptsx_local, ptsy_local, coeffs_);

        v += result[1] * dt; // <- simulate updated velocity

        // Publish the result
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = result[0]; // delta (steering)
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
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}
