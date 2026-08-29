#include "f1tenth_control/mpc.h"
#include "f1tenth_control/utils.h"
#include <Eigen/Dense>
#include <Eigen/QR>
#include <IpIpoptApplication.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Model Predictive Controller for Slash 2WD
class MPCNode : public rclcpp::Node {
  public:
    MPCNode() : Node("slash_mpc") {
        const auto centerline_csv = this->declare_parameter<std::string>("centerline_csv");
        base_frame_ = this->declare_parameter<std::string>("base_frame");
        max_forward_range_ = this->declare_parameter<double>("path.max_forward_range");
        const auto max_waypoints = this->declare_parameter<int>("path.max_waypoints");
        const auto min_waypoints = this->declare_parameter<int>("path.min_waypoints");
        visualization_enabled_ = this->declare_parameter<bool>("visualization.enabled");
        marker_point_size_ = this->declare_parameter<double>("visualization.point_size");
        marker_line_width_ = this->declare_parameter<double>("visualization.line_width");
        marker_path_length_ = this->declare_parameter<double>("visualization.path_length");
        marker_sample_step_ = this->declare_parameter<double>("visualization.sample_step");

        const auto horizon_steps = this->declare_parameter<int>("mpc.horizon_steps");
        MPCConfig mpc_config{
            static_cast<size_t>(horizon_steps),
            this->declare_parameter<double>("mpc.timestep"),
            this->declare_parameter<double>("vehicle.wheelbase"),
            this->declare_parameter<double>("mpc.reference_velocity"),
            this->declare_parameter<double>("mpc.weights.cte"),
            this->declare_parameter<double>("mpc.weights.heading_error"),
            this->declare_parameter<double>("mpc.weights.velocity"),
            this->declare_parameter<double>("mpc.weights.steering"),
            this->declare_parameter<double>("mpc.weights.acceleration"),
            this->declare_parameter<double>("mpc.weights.steering_rate"),
            this->declare_parameter<double>("mpc.weights.acceleration_rate"),
            this->declare_parameter<double>("limits.max_steering_angle"),
            this->declare_parameter<double>("limits.min_acceleration"),
            this->declare_parameter<double>("limits.max_acceleration"),
            this->declare_parameter<double>("limits.min_velocity"),
            this->declare_parameter<double>("limits.max_velocity"),
            this->declare_parameter<double>("mpc.solver_max_cpu_time")
        };

        validateParameters(centerline_csv, horizon_steps, max_waypoints, min_waypoints, mpc_config);
        max_waypoints_ = static_cast<size_t>(max_waypoints);
        min_waypoints_ = static_cast<size_t>(min_waypoints);
        loadCenterline(centerline_csv);
        mpc_ = std::make_unique<MPC>(mpc_config);

        // Subscribers
        // IsaacSim-provided odometry for now without any estimation
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { this->odomCallback(msg); }
        );

        // Publishers
        // Velocity commands from the MPC controller
        cmd_pub_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);

        if (visualization_enabled_) {
            // Green markers to visualize the local waypoints
            centerline_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
                "mpc/centerline_points",
                10
            );
            // Red markers to visualize the fitted path
            fit_marker_pub_ =
                this->create_publisher<visualization_msgs::msg::Marker>("mpc/polyfit", 10);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "MPC Node initialized with centerline '%s'",
            centerline_csv.c_str()
        );
    }

  private:
    void validateParameters(
        const std::string &centerline_csv,
        int horizon_steps,
        int max_waypoints,
        int min_waypoints,
        const MPCConfig &mpc_config
    ) const {
        if (centerline_csv.empty()) {
            throw std::invalid_argument("Parameter 'centerline_csv' must not be empty");
        }
        if (base_frame_.empty()) {
            throw std::invalid_argument("Parameter 'base_frame' must not be empty");
        }
        if (horizon_steps < 3) {
            throw std::invalid_argument("Parameter 'mpc.horizon_steps' must be at least 3");
        }
        if (min_waypoints < 4 || max_waypoints < min_waypoints) {
            throw std::invalid_argument(
                "Path waypoint parameters must satisfy 4 <= min_waypoints <= max_waypoints"
            );
        }
        if (max_forward_range_ <= 0.0 || marker_point_size_ <= 0.0 || marker_line_width_ <= 0.0 ||
            marker_path_length_ <= 0.0 || marker_sample_step_ <= 0.0) {
            throw std::invalid_argument("Distance and visualization parameters must be positive");
        }
        if (mpc_config.timestep <= 0.0 || mpc_config.wheelbase <= 0.0 ||
            mpc_config.solver_max_cpu_time <= 0.0) {
            throw std::invalid_argument("MPC timestep, wheelbase, and CPU time must be positive");
        }
        if (mpc_config.weight_cte < 0.0 || mpc_config.weight_heading_error < 0.0 ||
            mpc_config.weight_velocity < 0.0 || mpc_config.weight_steering < 0.0 ||
            mpc_config.weight_acceleration < 0.0 || mpc_config.weight_steering_rate < 0.0 ||
            mpc_config.weight_acceleration_rate < 0.0) {
            throw std::invalid_argument("MPC cost weights must not be negative");
        }
        if (mpc_config.min_acceleration > mpc_config.max_acceleration ||
            mpc_config.min_velocity > mpc_config.max_velocity ||
            mpc_config.max_steering_angle <= 0.0 ||
            mpc_config.reference_velocity < mpc_config.min_velocity ||
            mpc_config.reference_velocity > mpc_config.max_velocity) {
            throw std::invalid_argument("Invalid vehicle control limits");
        }
    }

    struct CarState {
        double px;
        double py;
        double psi;
        double v;
    };

    struct LocalWaypoints {
        std::vector<double> ptsx;
        std::vector<double> ptsy;
    };

    struct PathFit {
        Eigen::VectorXd coeffs;
        double cte;
        double epsi;
    };

    CarState getCurrentState(const nav_msgs::msg::Odometry::SharedPtr msg) const {
        // Phase 1 (Get current state): Get the car's current state
        // Get x, y position states from odometry msg
        // Refers to the car's x/y position in the global frame (same as centerline)
        const double px = msg->pose.pose.position.x;
        const double py = msg->pose.pose.position.y;

        // Get yaw from the orientation quaternion
        // psi: which way the car is facing (yaw)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        double roll, pitch, psi;
        tf2::Matrix3x3(q).getRPY(roll, pitch, psi);

        // Current forward speed from odometry (m/s)
        const double v = msg->twist.twist.linear.x;

        return CarState{px, py, psi, v};
    }

    LocalWaypoints findLocalWaypoints(const CarState &current_state) const {
        const double px = current_state.px;
        const double py = current_state.py;
        const double psi = current_state.psi;

        // Locate the closest waypoint to the current pose
        size_t closest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < ptsx_.size(); ++i) {
            double dx = ptsx_[i] - px; // difference in x between waypoint and car
            double dy = ptsy_[i] - py; // difference in y between waypoint and car
            // squared distance between waypoint and car
            // note: computing squared distance instead of distance to avoid square root operation
            // on each iteration
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq <
                min_dist_sq) // if current waypoint is closer than the previous closest waypoint
            {
                min_dist_sq = dist_sq; // update the closest waypoint
                closest_idx = i;       // update the index of the closest waypoint
            }
        }

        std::vector<double> ptsx_local; // local waypoints in the car's body frame
        std::vector<double> ptsy_local; // local waypoints in the car's body frame
        ptsx_local.reserve(max_waypoints_);
        ptsy_local.reserve(max_waypoints_);

        size_t samples_checked = 0;
        while (ptsx_local.size() < max_waypoints_ && samples_checked < ptsx_.size()) {
            size_t idx = (closest_idx + samples_checked) %
                         ptsx_.size(); // needed because the track is circular
            samples_checked++;

            double dx = ptsx_[idx] - px; // difference in x between waypoint and car
            double dy = ptsy_[idx] - py; // difference in y between waypoint and car

            double x_local =
                dx * cos(-psi) -
                dy * sin(-psi); // get the x position of the waypoint in the car's body frame
            double y_local =
                dx * sin(-psi) +
                dy * cos(-psi); // get the y position of the waypoint in the car's body frame

            if (x_local < 0.0 || x_local > max_forward_range_) {
                continue;
            }

            // vectors representing the forward (x) and lateral (y) distance to waypoint[i]
            // each poseCallback call results in a new set of local waypoints for that time step
            ptsx_local.push_back(x_local);
            ptsy_local.push_back(y_local);
        }

        if (ptsx_local.size() < min_waypoints_) {
            static rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                throttle_clock,
                1000,
                "Insufficient forward waypoints (%zu) for polyfit",
                ptsx_local.size()
            );
        }

        return LocalWaypoints{ptsx_local, ptsy_local};
    }

    PathFit fitToPath(const LocalWaypoints &local_waypoints) const {
        // y = Ac
        const Eigen::VectorXd ptsx_eigen = Eigen::Map<const Eigen::VectorXd>(
            local_waypoints.ptsx.data(),
            static_cast<Eigen::Index>(local_waypoints.ptsx.size())
        );
        const Eigen::VectorXd ptsy_eigen = Eigen::Map<const Eigen::VectorXd>(
            local_waypoints.ptsy.data(),
            static_cast<Eigen::Index>(local_waypoints.ptsy.size())
        );
        Eigen::VectorXd coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);
        double cte = polyeval(coeffs, 0.0); // y error at x=0
        double epsi = -std::atan(
            coeffs[1]
        ); // heading error at x=0 (the slope of the polynomial at x=0). It will always be x=0
           // because the car is always at the origin in the body frame.
        return PathFit{coeffs, cte, epsi};
    }

    void loadCenterline(const std::string &path) {

        std::ifstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open centerline CSV: " + path);
        }
        std::string line;
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str)) {
                try {
                    ptsx_.push_back(std::stod(x_str));
                    ptsy_.push_back(std::stod(y_str));
                } catch (const std::invalid_argument &e) {
                    RCLCPP_WARN(this->get_logger(), "Skipping invalid row: '%s'", line.c_str());
                }
            }
        }

        if (ptsx_.size() < min_waypoints_) {
            throw std::runtime_error(
                "Centerline CSV contains fewer usable rows than path.min_waypoints"
            );
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Phase 1 (Get current state): Get the car's current state
        const CarState current_state = getCurrentState(msg);
        double v = current_state.v;

        // Phase 2 (Localize/Fit the path): Find the closest waypoint and return the local waypoints
        LocalWaypoints local_waypoints = findLocalWaypoints(current_state);
        if (local_waypoints.ptsx.size() < min_waypoints_) {
            return;
        }
        PathFit path_fit = fitToPath(local_waypoints);

        // Setup state vector: [x, y, psi, v, cte, epsi]
        // cte: answers is the car to the right or left of the path
        // epsi: answers if the car's hood facing the same way as the path
        Eigen::VectorXd state(6);
        state << 0.0, 0.0, 0.0, v, path_fit.cte, path_fit.epsi;

        // Phase 3 (Solve for the optimal control inputs)
        MPCResult result = mpc_->Solve(state, path_fit.coeffs);
        if (!result.success) {
            static rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
            RCLCPP_ERROR_THROTTLE(this->get_logger(), throttle_clock, 1000, "MPC failed to solve");
            return;
        }
        publishVisualization(local_waypoints, path_fit);

        // Phase 4 (Publish the result)
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = msg->header.stamp;
        drive_msg.header.frame_id = base_frame_;
        drive_msg.drive.steering_angle = result.steering_angle;
        drive_msg.drive.speed = result.velocity;
        drive_msg.drive.acceleration = result.acceleration;
        cmd_pub_->publish(drive_msg);
    }

    void publishVisualization(const LocalWaypoints &local_waypoints, const PathFit &path_fit) {
        if (!centerline_marker_pub_ || !fit_marker_pub_) {
            return;
        }

        visualization_msgs::msg::Marker centerline_marker;
        centerline_marker.header.frame_id = base_frame_;
        centerline_marker.header.stamp = this->now();
        centerline_marker.ns = "mpc_centerline";
        centerline_marker.id = 0;
        centerline_marker.type = visualization_msgs::msg::Marker::POINTS;
        centerline_marker.action = visualization_msgs::msg::Marker::ADD;
        centerline_marker.scale.x = marker_point_size_;
        centerline_marker.scale.y = marker_point_size_;
        centerline_marker.color.g = 1.0f;
        centerline_marker.color.a = 1.0f;

        for (size_t i = 0; i < local_waypoints.ptsx.size(); ++i) {
            geometry_msgs::msg::Point p;
            p.x = local_waypoints.ptsx[i];
            p.y = local_waypoints.ptsy[i];
            centerline_marker.points.push_back(p);
        }

        visualization_msgs::msg::Marker fit_marker;
        fit_marker.header.frame_id = base_frame_;
        fit_marker.header.stamp = this->now();
        fit_marker.ns = "mpc_polyfit";
        fit_marker.id = 0;
        fit_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        fit_marker.action = visualization_msgs::msg::Marker::ADD;
        fit_marker.scale.x = marker_line_width_;
        fit_marker.color.r = 1.0f;
        fit_marker.color.a = 1.0f;

        for (double x = 0.0; x <= marker_path_length_; x += marker_sample_step_) {
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = polyeval(path_fit.coeffs, x);
            fit_marker.points.push_back(p);
        }

        centerline_marker_pub_->publish(centerline_marker);
        fit_marker_pub_->publish(fit_marker);
    }

    std::unique_ptr<MPC> mpc_;
    std::vector<double> ptsx_;
    std::vector<double> ptsy_;

    std::string base_frame_;
    double max_forward_range_;
    size_t max_waypoints_;
    size_t min_waypoints_;
    bool visualization_enabled_;
    double marker_point_size_;
    double marker_line_width_;
    double marker_path_length_;
    double marker_sample_step_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerline_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fit_marker_pub_;
};

int main(int argc, char **argv) {
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);
    // Construct the MPCNode and spin until shutdown
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}
