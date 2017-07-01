#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
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
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

double deriv_eval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); ++i) {
    result += i * coeffs[i] * pow(x, (i - 1));
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          double px = j[1]["x"]; // meters
          double py = j[1]["y"]; // meters
          double psi = j[1]["psi"]; // radians
          double v_mph = j[1]["speed"]; // mph

          // TODO: convert to meters
          double v_ms = v_mph * 0.44704; // now in meters/sec

          int n = ptsx.size();
          Eigen::VectorXd ptsx_(n);
          Eigen::VectorXd ptsy_(n);
          // convert to vehicle coordinate system
          for (int i = 0; i < ptsx.size(); ++i) {
            double d_x = ptsx[i]-px;
            double d_y = ptsy[i]-py;
            ptsx_[i] = d_x * cos(psi) + d_y * sin(psi);
            ptsy_[i] = -d_x * sin(psi) + d_y * cos(psi);
          }

          // order 3 polynomial
          auto coeffs = polyfit(ptsx_, ptsy_, 3);

          //Display the waypoints/reference line
          // YELLOW
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_inc = 2.5;
          int num_points = 25;
          for (int i = 1; i < num_points; i++)
          {
            next_x_vals.push_back(poly_inc*i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
          }

          // polyeal and deriv eval at point 0
          double cte = coeffs[0];
          double epsi = -atan(coeffs[1]);

          // compensation for latency
          const double latency = 0.1; // 100 ms
          const double Lf = 2.67;
          double steering_angle = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];

          // steering_angle = steering_angle * deg2rad(25); // correction for simulator

          double latency_distance = v_ms * latency;
          double delayed_x = latency_distance; // assuming linear approximation for distance, which is not correct
          double delayed_y = 0;
          double delayed_psi = - steering_angle * latency_distance / Lf; // latency_distance / Lf * ///  steering_angle * deg2rad(25) * latency
          double delayed_v = v_ms + throttle * latency;
          double delayed_cte = cte + latency_distance  * sin(epsi);
          double delayed_epsi = epsi + steering_angle;

          // fill the state in vehicle coordinates, so position and angle is always zero
          Eigen::VectorXd state(6);
          state << delayed_x, delayed_y, delayed_psi, delayed_v, delayed_cte, delayed_epsi;

          auto vars = mpc.Solve(state, coeffs);

          double steer_value = vars[0] / deg2rad(25); // from rad to [-1 1]
          double throttle_value = vars[1];

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          // GREEN
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (int i = 2; i < vars.size();)
          {
            mpc_x_vals.push_back(vars[i++]);
            mpc_y_vals.push_back(vars[i++]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

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
