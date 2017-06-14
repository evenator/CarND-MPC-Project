#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC(size_t n, double v, size_t delay_steps);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  const size_t N;

 private:
  /// Desired velocity to maintain
  double velocity;
  /// How many steps the system lags
  size_t delay;
};

#endif /* MPC_H */
