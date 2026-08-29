#ifndef F1TENTH_CONTROL_MPC_H
#define F1TENTH_CONTROL_MPC_H

#include "Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cstddef>
#include <vector>

using CppAD::AD;

struct MPCConfig {
    std::size_t horizon_steps;
    double timestep;
    double wheelbase;
    double reference_velocity;
    double weight_cte;
    double weight_heading_error;
    double weight_velocity;
    double weight_steering;
    double weight_acceleration;
    double weight_steering_rate;
    double weight_acceleration_rate;
    double max_steering_angle;
    double min_acceleration;
    double max_acceleration;
    double min_velocity;
    double max_velocity;
    double solver_max_cpu_time;
};

struct MPCLayout {
    explicit MPCLayout(std::size_t horizon);

    std::size_t horizon_steps;
    std::size_t x_start;
    std::size_t y_start;
    std::size_t psi_start;
    std::size_t velocity_start;
    std::size_t cte_start;
    std::size_t heading_error_start;
    std::size_t steering_start;
    std::size_t acceleration_start;
    std::size_t variable_count;
    std::size_t constraint_count;
};

struct MPCResult {
    bool success;
    double steering_angle = 0.0;
    double acceleration = 0.0;
    double velocity = 0.0;
};

class FG_eval {
  public:
    // Constructor takes polynomial coefficients
    FG_eval(Eigen::VectorXd coeffs, MPCConfig config, MPCLayout layout);

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars);

  private:
    Eigen::VectorXd coeffs_;
    MPCConfig config_;
    MPCLayout layout_;
};

// Define the MPC controller class
class MPC {
  public:
    explicit MPC(MPCConfig config);
    virtual ~MPC() = default;

    // Solve method: takes current vehicle state and polynomial coefficients, returns control inputs
    MPCResult Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

    // Store predicted x and y trajectory for visualization
    std::vector<double> mpc_x_vals;
    std::vector<double> mpc_y_vals;

  private:
    MPCConfig config_;
    MPCLayout layout_;
};

#endif // F1TENTH_CONTROL_MPC_H
