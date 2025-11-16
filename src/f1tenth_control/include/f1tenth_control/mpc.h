#ifndef F1TENTH_CONTROL_MPC_H
#define F1TENTH_CONTROL_MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "Eigen/Core"

using CppAD::AD;

class FG_eval
{
public:
    // Constructor takes polynomial coefficients
    FG_eval(Eigen::VectorXd coeffs) : coeffs_(coeffs) {}

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars);

private:
    Eigen::VectorXd coeffs_;
};

// Define the MPC controller class
class MPC
{
public:
    MPC();
    virtual ~MPC() = default;

    // Solve method: takes current vehicle state and polynomial coefficients, returns control inputs
    std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

    // Store predicted x and y trajectory for visualization
    std::vector<double> mpc_x_vals;
    std::vector<double> mpc_y_vals;
};

// Timestep length and duration
const size_t N = 10;
const double dt = 0.1;

// Length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Variable indices for each state and actuator
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

#endif // F1TENTH_CONTROL_MPC_H
