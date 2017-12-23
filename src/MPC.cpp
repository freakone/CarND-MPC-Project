#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <math.h>

using CppAD::AD;

const int cte_multiplier = 2200;
const int x_start = 0;
const int y_start = x_start + MPC::N;
const int psi_start = y_start + MPC::N;
const int v_start = psi_start + MPC::N;
const int cte_start = v_start + MPC::N;
const int epsi_start = cte_start + MPC::N;
const int delta_start = epsi_start + MPC::N;
const int a_start = delta_start + MPC::N - 1;

class FG_eval {
    public:
        Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) {
        this->coeffs = coeffs;
    }

    typedef CPPAD_TESTVECTOR(AD < double > ) ADvector;
    void operator()(ADvector & fg,
        const ADvector & vars) {

        fg[0] = 0;

        for (int i = 0; i < MPC::N; i++) {
            fg[0] += cte_multiplier * CppAD::pow(vars[cte_start + i], 2);
            fg[0] += CppAD::pow(vars[epsi_start + i], 2);
            fg[0] += CppAD::pow(vars[v_start + i] - MPC::ref_v, 2);

        }

        for (int i = 0; i < MPC::N - 1; i++) {
            fg[0] += CppAD::pow(vars[delta_start + i], 2);
            fg[0] += CppAD::pow(vars[a_start + i], 2);
        }

        for (int i = 0; i < MPC::N - 2; i++) {
            fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
            fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
        }

        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (int t = 1; t < MPC::N; t++) {
            // The state at time t+1 .
            AD < double > x1 = vars[x_start + t];
            AD < double > y1 = vars[y_start + t];
            AD < double > psi1 = vars[psi_start + t];
            AD < double > v1 = vars[v_start + t];
            AD < double > cte1 = vars[cte_start + t];
            AD < double > epsi1 = vars[epsi_start + t];

            // The state at time t.
            AD < double > x0 = vars[x_start + t - 1];
            AD < double > y0 = vars[y_start + t - 1];
            AD < double > psi0 = vars[psi_start + t - 1];
            AD < double > v0 = vars[v_start + t - 1];
            AD < double > cte0 = vars[cte_start + t - 1];
            AD < double > epsi0 = vars[epsi_start + t - 1];

            // Only consider the actuation at time t.
            AD < double > delta0 = vars[delta_start + t - 1];
            AD < double > a0 = vars[a_start + t - 1];

            AD < double > f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
            AD < double > psides0 = CppAD::atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3] * CppAD::pow(x0, 2)));

            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * MPC::dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * MPC::dt);
            fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / MPC::Lf * MPC::dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * MPC::dt);
            fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * MPC::dt));
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / MPC::Lf * MPC::dt);
        }

    }
};

MPC::MPC() {}
MPC::~MPC() {}

MPCResult MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    // State: [x,y,ψ,v,cte,eψ]
    // Actuators: [δ,a]
    const int n_vars = N * 6 + (N - 1) * 2;
    // Number of constraints
    const int n_constraints = N * 6;

    const double x = state[0];
    const double y = state[1];
    const double psi = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double epsi = state[5];

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0.0;
    }

    // Set initial variable values to init state
    // but will be implicitly set by upper and lower bounds later so can remove this
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    // Steering angle (deltas)
    for (int i = delta_start; i < a_start; i++) {
        vars_upperbound[i] = M_PI / 8; // max values allowed in simulator
        vars_lowerbound[i] = -M_PI / 8;
    }
    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    // Set init state lower and upper limits
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // options
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result < Dvector > solution;

    // solve the problem
    CppAD::ipopt::solve < Dvector, FG_eval > (
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    //
    // Check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result < Dvector > ::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    MPCResult result;
    result.angle = solution.x[delta_start];
    result.throttle = solution.x[a_start];

    for (int i = 0; i < N - 1; i++) {
        result.x_prediction.push_back(solution.x[x_start + i + 1]);
        result.y_prediction.push_back(solution.x[y_start + i + 1]);
    }

    return result;
}

std::vector < double > MPC::toStdVector(const Eigen::VectorXd & input) {
    std::vector < double > result;

    for (int i = 0; i < input.size(); i++) {
        result.emplace_back(input(i));
    }

    return result;
}

Eigen::MatrixXd MPC::transformCoordinates(const vector < double > & vx,
    const vector < double > & vy,
        const double & px,
            const double & py,
                const double & psi) {
    if (vx.size() != vy.size()) {
        return Eigen::MatrixXd();
    }

    Eigen::MatrixXd result(2, vy.size());

    for (size_t i = 0; i < vx.size(); i++) {
        const double dx = vx[i] - px;
        const double dy = vy[i] - py;
        result(0, i) = dx * cos(psi) + dy * sin(psi);
        result(1, i) = -dx * sin(psi) + dy * cos(psi);
    }

    return result;
}