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
const double Lf = 2.67;
double latency = 0.1;

//Display the waypoints/reference line
vector<double> next_x_vals;
vector<double> next_y_vals;
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
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
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

vector<double> CalculateNewXY(double px, double py, double psi, double ptsx, double ptsy)
{
  vector<double> output(2);

  output[0] = ((ptsx - px) * cos(psi)) + ((ptsy - py) * sin(psi));
  output[1] = ((ptsy - py) * cos(psi)) - ((ptsx - px) * sin(psi));

  next_x_vals.push_back(output[0]);
  next_y_vals.push_back(output[1]);
  return output;
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
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          size_t xy_size;
          double steer_value;
          double throttle_value;
          Eigen::VectorXd state(6);
          vector<double> next_x_y(2);
          double next_v, next_cte, next_epsi;
// ================================================================================
          next_x_vals.erase (next_x_vals.begin(),next_x_vals.end());
          next_y_vals.erase (next_y_vals.begin(),next_y_vals.end());
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];
          next_v = v + a * latency;

          // find the number of x and y co-ordinates.
          xy_size = ptsx.size();

          Eigen::VectorXd trackpoints_x(xy_size);
          Eigen::VectorXd trackpoints_y(xy_size);
          for (size_t m = 0; m < xy_size; ++m)
          {
            /* code */
            next_x_y = CalculateNewXY(px, py, psi, ptsx[m], ptsy[m]);
            trackpoints_x[m] = next_x_y[0];
            trackpoints_y[m] = next_x_y[1];
          }
          // The polynomial is fitted to a straight line so a polynomial with
          // order 2 is sufficient.
          Eigen::VectorXd coeffs = polyfit(trackpoints_x, trackpoints_y, 2);
          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          double cte = polyeval(coeffs, 0);
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = 0 - atan(coeffs[1]);

// https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/af4fcd4f-eb1f-43d8-82b3-17bb1e71695f/concepts/e88c2080-1abc-4800-a83c-83f52b2ca0c8
          next_cte = cte + v * sin(epsi) * latency;
          next_epsi = epsi - v / Lf * delta * latency;

          state << 0, 0, 0, next_v, next_cte, next_epsi;

          vector<double> vars = mpc.Solve(state, coeffs);
          int vars_len = (vars.size() - 2) / 2;
          for (int i = 0; i < vars_len - 2; ++i) {
            mpc_x_vals.push_back(vars[i + 2]);
            mpc_y_vals.push_back(vars[i + 2 + vars_len]);
          }

          steer_value = vars[0];
          throttle_value = vars[1];
// ================================================================================
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

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
