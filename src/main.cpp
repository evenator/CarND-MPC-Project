#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(std::vector<Eigen::VectorXd> const& points, int order) {
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

Eigen::VectorXd pointToVehicleFrame(Eigen::VectorXd const& world_pt,
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

json controlFromTelemetry(MPC& mpc, json& telemetry) {
  /// Order of the polynomial to fit to the waypoints
  const int FIT_ORDER = 3;

  vector<double> ptsx = telemetry["ptsx"];
  vector<double> ptsy = telemetry["ptsy"];
  double px = telemetry["x"];
  double py = telemetry["y"];
  double psi = telemetry["psi"];
  double v = telemetry["speed"];

  vector<Eigen::VectorXd> waypoints;
  Eigen::VectorXd vehicle_pos(2);
  vehicle_pos << px, py;
  waypoints.reserve(ptsx.size());
  for (size_t i = 0; i < ptsx.size(); ++i) {
    Eigen::VectorXd world_pt(2);
    world_pt << ptsx[i], ptsy[i];
    waypoints.push_back(pointToVehicleFrame(world_pt, vehicle_pos, psi));
  }

  Eigen::VectorXd poly_coeffs = polyfit(waypoints, FIT_ORDER);
  double cte = poly_coeffs[0];
  double yaw_err = atan(poly_coeffs[1]);

  Eigen::VectorXd state(6);
  state << px, py, psi, v, cte, yaw_err;

  auto results = mpc.Solve(state, poly_coeffs);

  /*
   * TODO: Calculate steeering angle and throttle using MPC.
   *
   * Both are in between [-1, 1].
   *
   */
  double steer_value = 0;
  double throttle_value = 0;

  json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = throttle_value;

  //Display the MPC predicted trajectory
  vector<double> mpc_x_vals(mpc.N);
  vector<double> mpc_y_vals(mpc.N);

  //TODO: add (x,y) points to list here, points are in reference to the vehicle's coordinate system
  // the points in the simulator are connected by a Green line
  std::copy(results.begin()+2, results.begin()+2+mpc.N, mpc_x_vals.begin());
  std::copy(results.begin()+2+mpc.N, results.end(), mpc_y_vals.begin());
  
  msgJson["mpc_x"] = mpc_x_vals;
  msgJson["mpc_y"] = mpc_y_vals;

  //Display the waypoints/reference line
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  for (auto const& wp : waypoints) {
    next_x_vals.push_back(wp.x());
    next_y_vals.push_back(wp.y());
  }

  msgJson["next_x"] = next_x_vals;
  msgJson["next_y"] = next_y_vals;
  return msgJson;
}

int main() {

  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc(25, 40);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
      string sdata = string(data).substr(0, length);
      cout << sdata << endl;
      if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
        string s = hasData(sdata);
        if (s != "") {
          auto j = json::parse(s);
          string event = j[0].get<string>();
          if (event == "telemetry") {
            auto response = controlFromTelemetry(mpc, j[1]);

            auto msg = "42[\"steer\"," + response.dump() + "]";
            std::cout << msg << std::endl;
            // Latency
            // The purpose is to mimic real driving conditions where
            // the car does actuate the commands instantly.
            //
            // Feel free to play around with this value but should be to drive
            // around the track with 100ms latency.
            //
            // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
            // SUBMITTING.
            this_thread::sleep_for(chrono::milliseconds(100));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
    });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
