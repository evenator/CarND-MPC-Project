#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC(size_t n, double time_step, double v, size_t delay_steps);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  /// Number of time steps to predict
  const size_t N;
  /// Time step for prediction
  const double dt;
  /// How many steps the system lags
  const size_t delay;

 private:
  /// Desired velocity to maintain
  double velocity;
};

#endif /* MPC_H */
