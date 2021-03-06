#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "utility.h"

using CppAD::AD;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  /// Desired velocity
  double v_des;
  /// Number of states to predict
  size_t N;
  /// Timestep
  double dt;

  FG_eval(Eigen::VectorXd coeffs, double velocity, size_t n, double dt) {
    this->coeffs = coeffs;
    this->v_des = velocity;
    this->N = n;
    this->dt = dt;
  }

  /*
   * Layout of vars (State 0:N and actuators 1:N)
   * 0:N        x
   * N:2N       y
   * 2N:3N      psi
   * 3N:4N      velocity
   * 4N:5N      cross track error
   * 5N:6N      orientation error
   * 6N:7N-1    steering angle
   * 7N-1:8N-2 acceleration
   *
   * Layout of fg (cost + constraints):
   * 0         cost
   * 1:N+1     x
   * N+1:2N+1  y
   * 2N+1:3N+1 psi
   * 3N+1:4N+1 velocity
   * 4N+1:5N+1 cross track error
   * 5N+1:6N+1 orientation error
   */
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost function
    AD<double> cost = 0;
    // Penalize error
    for (size_t n = 0; n < N; ++n) {
      // Cross-track error
      cost += 1.2 * CppAD::pow(vars[4*N + n], 2);
      // Orientation error
      cost += 120 * CppAD::pow(vars[5*N + n], 2);
      // Velocity error
      cost += 5 * CppAD::pow(vars[3*N + n] - v_des, 2);
    }
    // Penalize actuation

    for (size_t n = 0; n < N - 1; ++n) {
      // Steering
      //cost += 5 * CppAD::pow(vars[6*N + n], 2);
      // Throttle
      //cost += CppAD::pow(vars[7*N - 1 + n], 2);
    }
    // Penalize rapid change in actuation
    for (size_t n = 0; n < N - 2; ++n) {
      // Steering
      cost += 0.05 * CppAD::pow(vars[6*N+n+1] - vars[6*N+n], 2);
      // Throttle
      cost += CppAD::pow(vars[7*N + n] - vars[7*N - 1 + n], 2);
    }
    fg[0] = cost;

    // Initial state constraints
    fg[1] = vars[0];  // x
    fg[1 + 1 * N] = vars[1 * N];  // y
    fg[1 + 2 * N] = vars[2 * N];  // psi
    fg[1 + 3 * N] = vars[3 * N];  // v
    fg[1 + 4 * N] = vars[4 * N];  // cross track error
    fg[1 + 5 * N] = vars[5 * N];  // orientation error

    // Kinematic state constraints for each time step
    for (size_t n = 0; n < N - 1; ++n) {
      // The state at time t
      AD<double> x_0 = vars[n];
      AD<double> y_0 = vars[N + n];
      AD<double> psi_0 = vars[2*N + n];
      AD<double> v_0 = vars[3*N + n];
      AD<double> cte_0 = vars[4*N + n];
      AD<double> yaw_err_0 = vars[5*N + n];
      AD<double> fx_0 = coeffs[0];
      for (size_t i = 2; i < coeffs.size(); ++i) {
        AD<double> x_pow = x_0;
        for (size_t j = 1; j < i; ++j) {
          x_pow *= x_0;
        }
        fx_0 += coeffs[i] * x_pow;
      }

      // The state at time t+1
      AD<double> x_1 = vars[n + 1];
      AD<double> y_1 = vars[N + n + 1];
      AD<double> psi_1 = vars[2*N + n + 1];
      AD<double> v_1 = vars[3*N + n + 1];
      AD<double> cte_1 = vars[4*N + n + 1];
      AD<double> yaw_err_1 = vars[5*N + n + 1];

      // The actuations at time t+1
      AD<double> steering = vars[6 * N + n];
      AD<double> accel = vars[7 * N - 1 + n];

      // The desired heading at time t
      // Heading is equal to arctangent of the slope
      AD<double> slope = 0;
      for (size_t i = 1; i < coeffs.size(); ++i) {
        AD<double> x_pow = 1;
        for (size_t j = 1; j < i; ++j) {
          x_pow *= x_0;
        }
        slope += i * coeffs[i] * x_pow;
      }
      AD<double> psi_des = CppAD::atan(slope);

      // Kinematic state constraints
      // 0 = x' - (x + v * cos(psi) * dt);
      fg[2 + n] = x_1 - (x_0 + v_0 * CppAD::cos(psi_0) * dt);
      // 0 = y' - (y + v * sin(psi) * dt);
      fg[2 + N + n] =  y_1 - (y_0 + v_0 * CppAD::sin(psi_0) * dt);
      // 0 = psi' - (psi - v * Lf * d_steering * dt)
      fg[2 + 2*N + n] = psi_1 - (psi_0 - v_0 * steering / Lf * dt);
      // 0 = v' - (v + a * dt);
      fg[2 + 3*N + n] = v_1 - (v_0 + accel * dt);
      // 0 = cte' - (f(x) - y + v * sin(yaw_err) * dt)
      fg[2 + 4*N + n] = cte_1 - (fx_0 - y_0 + v_0 * CppAD::sin(yaw_err_0) * dt);
      // 0 = yaw_err' - (psi - psi_des - v * d_steering / Lf * dt)
      fg[2 + 5*N + n] = yaw_err_1 - (psi_1 - psi_des);
    }
  }
};

MPC::MPC(
  size_t n,
  double time_step,
  double v,
  size_t delay_steps):
    N(n),
    dt(time_step),
    velocity(v),
    delay(delay_steps) {}

MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  const size_t n_vars = state.size() * N + N_ACTUATORS * (N - 1);
  // Set the number of constraints
  const size_t n_constraints = state.size() * N;

  // Initial value of the independent variables.
  Dvector vars(n_vars);
  // SHOULD BE 0 besides initial state.
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  // Initial state
  for (size_t i = 0; i < state.size(); ++i) {
    vars[i * N] = state[i];
  }

  // Variable Bounds
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set non-actuator bounds to an arbitrary large range
  for (int n = 0; n < state.size()*N; ++n) {
    // x
    vars_lowerbound[n] = -1e19;
    vars_upperbound[n] = 1e19;
  }
  // Set velocity bounds to 0-100
  for (int n = 0; n < N; ++n)
  {
    vars_lowerbound[3*N + n] = 0;
    vars_upperbound[3*N + n] = 100;
  }

  // Constrain Steering During Acuation Delay
  for (size_t n = 0; n < delay; ++n) {
    vars_lowerbound[6*N + n] = state[7];
    vars_upperbound[6*N + n] = state[7];
  }
  // Constrain throttle during actuation delay
  for (size_t n = 0; n < delay; ++n) {
    vars_lowerbound[7*N - 1 + n] = state[6];
    vars_upperbound[7*N - 1 + n] = state[6];
  }

  // Set steering bounds to +/- MAX_STEERING_ANGLE radians
  for (int n = delay; n < N-1; ++n) {
    vars_lowerbound[6*N + n] = -MAX_STEERING_ANGLE;
    vars_upperbound[6*N + n] = MAX_STEERING_ANGLE;
  }
  // Set throttle bounds to +/- MAX_ACCELERATION
  for (int n = delay; n < N-1; ++n) {
    vars_lowerbound[7*N - 1 + n] = -MAX_ACCELERATION;
    vars_upperbound[7*N - 1 + n] = MAX_ACCELERATION;
  }

  // Constraint bounds
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  // Should be 0 besides initial state.
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // Initial state is fixed
  for (size_t i = 0; i < state.size(); ++i) {
    constraints_lowerbound[i * N] = state[i];
    constraints_upperbound[i * N] = state[i];
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, velocity, N, dt);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Comment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can comment 1 of these and see if it makes a difference or not but
  // if you comment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  
  // Return first steering, first throttle, then full x sequence, full y sequence
  std::vector<double> results;
  results.push_back(solution.x[6*N+delay]);  // Steering
  results.push_back(solution.x[7*N-1+delay]);  // Throttle
  for (size_t i = delay; i < N; ++i) {
    results.push_back(solution.x[i]);
  }
  for (size_t i = N+delay; i < 2*N; ++i) {
    results.push_back(solution.x[i]);
  }
  return results;
}
