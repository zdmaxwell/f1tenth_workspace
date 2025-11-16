#include "f1tenth_control/mpc.h"
#include <cppad/ipopt/solve.hpp>
#include <iostream>

// FG_eval::operator() — sets up the cost and dynamics constraints
void FG_eval::operator()(ADvector &fg, const ADvector &vars)
{

    assert(coeffs_.size() == 4 && "Expected 4 polynomial coefficients!");

    // Cost is stored in fg[0]
    fg[0] = 0;

    // Weights for different parts of the cost function
    const double w_cte = 2000;
    const double w_epsi = 2000;
    const double w_v = 10000;
    const double w_delta = 10;
    const double w_a = 10;
    const double w_ddelta = 100;
    const double w_da = 10;

    // Reference velocity
    const double ref_v = 2.5;

    // Cost terms
    for (size_t t = 0; t < N - 1; t++)
    {
        fg[0] += w_cte * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += w_epsi * CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += w_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    for (size_t t = 0; t < N - 1; t++)
    {
        fg[0] += w_delta * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += w_a * CppAD::pow(vars[a_start + t], 2);
    }

    for (size_t t = 0; t < N - 2; t++)
    {
        fg[0] += w_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += w_da * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initial constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Apply the vehicle model constraints
    for (size_t t = 0; t < N - 1; t++)
    {
        AD<double> x0 = vars[x_start + t];
        AD<double> y0 = vars[y_start + t];
        AD<double> psi0 = vars[psi_start + t];
        AD<double> v0 = vars[v_start + t];
        AD<double> cte0 = vars[cte_start + t];
        AD<double> epsi0 = vars[epsi_start + t];

        AD<double> delta0 = vars[delta_start + t];
        AD<double> a0 = vars[a_start + t];

        AD<double> x1 = vars[x_start + t + 1];
        AD<double> y1 = vars[y_start + t + 1];
        AD<double> psi1 = vars[psi_start + t + 1];
        AD<double> v1 = vars[v_start + t + 1];
        AD<double> cte1 = vars[cte_start + t + 1];
        AD<double> epsi1 = vars[epsi_start + t + 1];

        AD<double> f0 = coeffs_[0] + coeffs_[1] * x0 + coeffs_[2] * CppAD::pow(x0, 2) + coeffs_[3] * CppAD::pow(x0, 3);
        AD<double> psides0 = CppAD::atan(coeffs_[1] + 2 * coeffs_[2] * x0 + 3 * coeffs_[3] * CppAD::pow(x0, 2));

        // Kinematic equations
        fg[1 + x_start + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t + 1] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
        fg[1 + v_start + t + 1] = v1 - (v0 + a0 * dt);
        fg[1 + cte_start + t + 1] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
        fg[1 + epsi_start + t + 1] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
}

MPC::MPC() {}

// MPC::Solve — sets up and solves the optimization problem
std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    typedef CPPAD_TESTVECTOR(double) Dvector;

    size_t n_vars = N * 6 + (N - 1) * 2;
    size_t n_constraints = N * 6;

    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++)
        vars[i] = 0;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    // Variable bounds
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    for (size_t i = 0; i < delta_start; i++)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    for (size_t i = delta_start; i < a_start; i++)
    {
        vars_lowerbound[i] = -0.436332; // -25 deg in radians
        vars_upperbound[i] = 0.436332;
    }

    for (size_t i = a_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -0.5;
        vars_upperbound[i] = 1.0;
    }
    for (size_t i = 0; i < N; i++)
    {
        vars_lowerbound[v_start + i] = 0.0; // prevent reverse
        vars_upperbound[v_start + i] = 2.5;
    }

    // Constraint bounds
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (size_t i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound = constraints_lowerbound;

    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // Options for IPOPT
    std::string options;
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    // Solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound,
        constraints_lowerbound, constraints_upperbound,
        fg_eval, solution);

    // Store predicted trajectory
    mpc_x_vals.clear();
    mpc_y_vals.clear();
    for (size_t t = 0; t < N; t++)
    {
        mpc_x_vals.push_back(solution.x[x_start + t]);
        mpc_y_vals.push_back(solution.x[y_start + t]);
    }

    return {
        solution.x[delta_start], // steering angle
        solution.x[a_start],     // acceleration
        solution.x[v_start]      // velocity
    };
}

// #include <cppad/cppad.hpp>
// #include <cppad/ipopt/solve.hpp>
// #include <vector>
// #include <Eigen/Core>

// const size_t N = 10;
// const double dt = 0.1;

// const double Lf = 2.67;

// const size_t x_start = 0;
// const size_t y_start = x_start + N;
// const size_t psi_start = y_start + N;
// const size_t v_start = psi_start + N;
// const size_t cte_start = v_start + N;
// const size_t epsi_start = cte_start + N;
// const size_t delta_start = epsi_start + N;
// const size_t a_start = delta_start + N - 1;

// class FG_eval
// {
// public:
//     int goal_cte = 0;    // zero cross track error; Slash drives along the center of centerline path
//     int goal_epsi = 0;   // zero heading error; Slash's heading matches the path's tangent line
//     double goal_v = 1.0; // maintain 1 m/s speed

//     Eigen::VectorXd coeffs;

//     FG_eval(Eigen::VectorXd coeffs) : coeffs(coeffs) {}

//     typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

//     // fg array contains the cost at fg[0] and the constraints at fg[1:]
//     // vars is the candidate solution vector that comes from the solver
//     void operator()(ADvector &fg, const ADvector &vars)
//     {

//         fg[0] = 0;

//         for (size_t t = 0; t < N; t++)
//         {
//             // Accumulate penalties for the tracking errors between the current and goal states
//             fg[0] += CppAD::pow(vars[cte_start + t] - goal_cte, 2);
//             fg[0] += CppAD::pow(vars[epsi_start + t] - goal_epsi, 2);
//             fg[0] += CppAD::pow(vars[v_start + t] - goal_v, 2);
//         }

//         for (size_t t = 0; t < N; t++)
//         {
//             // Accumulate penalties for large control inputs (steering angle and acceleration)
//             fg[0] += CppAD::pow(vars[delta_start + t], 2);
//             fg[0] += CppAD::pow(vars[a_start + t], 2);
//         }

//         for (size_t t = 0; t < N; t++)
//         {
//             // Accumulate penalties for jerky control inputs (change in steering angle and acceleration)
//             fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
//             fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
//         }

//         // Set the initial state contraints for x, y, psi, v, cte, and epsi
//         fg[1 + x_start] = vars[x_start];
//         fg[1 + y_start] = vars[y_start];
//         fg[1 + psi_start] = vars[psi_start];
//         fg[1 + v_start] = vars[v_start];
//         fg[1 + cte_start] = vars[cte_start];
//         fg[1 + epsi_start] = vars[epsi_start];

//         // Get the kinematic bicycle model constraints for each time step
//         for (size_t t = 0; t < N - 1; t++)
//         {
//             // Get current state at time t
//             CppAD::AD<double> x0 = vars[x_start + t];
//             CppAD::AD<double> y0 = vars[y_start + t];
//             CppAD::AD<double> psi0 = vars[psi_start + t];
//             CppAD::AD<double> v0 = vars[v_start + t];
//             CppAD::AD<double> cte0 = vars[cte_start + t];
//             CppAD::AD<double> epsi0 = vars[epsi_start + t];

//             // Control at time t (steering angle and acceleration)
//             CppAD::AD<double> delta0 = vars[delta_start + t];
//             CppAD::AD<double> a0 = vars[a_start + t];

//             // Get the state at time t+1
//             CppAD::AD<double> x1 = vars[x_start + t + 1];
//             CppAD::AD<double> y1 = vars[y_start + t + 1];
//             CppAD::AD<double> psi1 = vars[psi_start + t + 1];
//             CppAD::AD<double> v1 = vars[v_start + t + 1];
//             CppAD::AD<double> cte1 = vars[cte_start + t + 1];
//             CppAD::AD<double> epsi1 = vars[epsi_start + t + 1];

//             // Get the desired y and heading on the path for the given x value at time t
//             CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
//             CppAD::AD<double> psiDes0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 + x0);

//             // Store difference (error) between the predicted state (at X_t+1) and the MPC solver chosen state (at X_t+1)
//             // MPC solver will adjust the state in vars[] to minimize these errors
//             // Goal is to minimize the residual to force the MPC solver to abide by the vehicle dynamics
//             fg[1 + x_start + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);                // x position update
//             fg[1 + y_start + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);                // y position update
//             fg[1 + psi_start + t + 1] = psi1 - (psi0 + v0 * delta0 / Lf * dt);               // heading update
//             fg[1 + v_start + t + 1] = v1 - (v0 + a0 * dt);                                   // velocity update
//             fg[1 + cte_start + t + 1] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);    // cross track error update
//             fg[1 + epsi_start + t + 1] = epsi1 - ((psi0 - psiDes0) + v0 * delta0 / Lf * dt); // heading error update
//         }
//     }
// };