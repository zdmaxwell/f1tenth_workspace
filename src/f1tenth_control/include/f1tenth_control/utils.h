#include <Eigen/Dense>

// Helper function to build an N order polynomial to fit the centerline path data
Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

    // A is the Vandermonde matrix of xvals (xvals # of rows, and order + 1 # of columns)
    Eigen::MatrixXd A(xvals.size(), order + 1); // e.g. [[1, x0, x0^2, ..., x0^order], [1, x1^1, x1^2, ..., x1^order]]

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
        for (int j = 1; j <= order; j++)
        {
            A(i, j) = A(i, j - 1) * xvals[i];
        }
    }

    auto Q = A.householderQr();
    Eigen::VectorXd result = Q.solve(yvals);
    return result;
}

inline double polyeval(const Eigen::VectorXd &coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); ++i)
    {
        result += coeffs[i] * std::pow(x, i);
    }
    return result;
}