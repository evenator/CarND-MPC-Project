#ifndef UTILITY_H_
#define UTILITY_H_

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// TODO: Set the timestep length and duration
const size_t N_ACTUATORS = 2;  // Throttle and steering
const double MAX_STEERING_ANGLE = 0.436332313;  // Radians
const double MAX_ACCELERATION = 3.0;  // m/s^2

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}

inline double deg2rad(double x) {
  return x * pi() / 180;
}

inline double rad2deg(double x) {
  return x * 180 / pi();
}

// Evaluate a polynomial.
static double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
static Eigen::VectorXd polyfit(std::vector<Eigen::VectorXd> const& points, int order) {
  // TODO: Reduce gratuitous copying in this function
  assert(order >= 1);
  assert(order <= points.size() - 1);
  Eigen::MatrixXd A(points.size(), order + 1);

  for (int i = 0; i < points.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < points.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * points[j].x();
    }
  }

  Eigen::VectorXd y_vals(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    y_vals(i) = points[i].y();
  }

  auto Q = A.householderQr();
  auto result = Q.solve(y_vals);
  return result;
}

inline double steeringAngleToRatio(double steering_angle) {
  return rad2deg(steering_angle)/(25.0 * Lf);
}

inline double steeringRatioToAngle(double steering_ratio) {
  return deg2rad(steering_ratio * 25.0 * Lf);
}

inline double mphToMps(double speed_mph) {
  return speed_mph * 0.44704;
}

inline double mpsToMph(double speed_mps) {
  return speed_mps / 0.44704;
}

inline double accelToThrottle(double accel_mpss) {
  if (accel_mpss > 0)  return accel_mpss / 3.3;  // Approximate
  else return accel_mpss / 6.7;  // Approximate
}

inline double throttleToAccel(double throttle) {
  if (throttle > 0)  return throttle * 3.3;  // Approximate
  else return throttle * 6.7;  // Approximate
}

static Eigen::VectorXd pointToVehicleFrame(Eigen::VectorXd const& world_pt,
                                    Eigen::VectorXd const& vehicle_pos,
                                    double psi) {
  Eigen::VectorXd trans = world_pt - vehicle_pos;
  // TODO: Implement using Eigen::Transform for speed and fanciness
  double r = trans.norm();
  double bearing = atan2(trans.y(), trans.x());

  Eigen::VectorXd vehicle_point(2);
  vehicle_point << r * cos(bearing - psi), r * sin(bearing - psi);
  return vehicle_point;
}

#endif
