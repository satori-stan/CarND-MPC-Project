#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
//#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helper_functions.h"
#include "MPC.h"

// for convenience
using json = nlohmann::json;
using namespace CarND;

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


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  CarND::MPC mpc;

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
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // The following values are used as approximations of process values
          // required to deal with latency. The steering angle is really the
          // angle-of-steer ratio based on the maximum steer angle (25°) so we
          // multiply it by that and flip the direction since the telemetry
          // value is sent with inverted sign. The same goes for the throttle:
          // empirically measured in the simulator, a maximum throttle of 1
          // makes the car reach 40mph in 4 seconds, so we multiply the throttle
          // times 10 to approximate the acceleration.
          double delta = j[1]["steering_angle"];
          delta *= -MPC::kMaxSteerAngle;
          double acceleration = j[1]["throttle"];
          acceleration *= 10;  // TODO: Make it a constant of the model as well


          // Shift the waypoints to ease calculation
          for (size_t i = 0; i < ptsx.size(); ++i) {
            // Translate the position
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;

            // Rotate the points, adjusting the direction
            ptsx[i] = x * cos(-psi) - y * sin(-psi);
            ptsy[i] = x * sin(-psi) + y * cos(-psi);
          }

          // Calculate cte and epsi. For this we first fit a third degree
          // polinomial to the waypoints, as suggested in the lectures.
          Eigen::VectorXd coefficients = HelperFunctions::PolyFit(
              Eigen::Map<Eigen::VectorXd>(&ptsx[0], ptsx.size()),
              Eigen::Map<Eigen::VectorXd>(&ptsy[0], ptsy.size()),
              3);
          // Then we evaluate the polinomial at the first point and subtract the
          // result to the current y value. This is again, an approximation of
          // the CTE.
          //double cte = py - HelperFunctions::PolyEval(coefficients, px);
          // This is the simplified form, with px and py = 0
          double cte = -coefficients[0];
          // Following is the calculation of error in psi, but since psi and px
          // are zero, we can simplify and use less computing cycles.
          /*
          // Notice we use a helper function to calculate the derivative. This
          // helps if the polinomial is of variable degrees.
          double epsi = psi - atan(HelperFunctions::PolyEval(
              HelperFunctions::Derivative(coefficients), px));
          */
          double epsi = -atan(coefficients[1]);

          // Calculate actuation values
          Eigen::VectorXd state(6); // TODO: Get rid of magic number
          double latency = 0.15;
          state <<
            0 + v * cos(0) * latency,
            0 + v * sin(0) * latency,
            0 + v * delta * latency / MPC::kLf,
            v + acceleration * latency,
            cte, epsi;

          MpcSolution control_actuation = mpc.Solve(state, coefficients);
          double steer_value = -1 * control_actuation.steering;  // Adjust steering direction
          double throttle_value = control_actuation.acceleration;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          next_x_vals = ptsx;
          next_y_vals = ptsy;

          json msgJson;

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          //double steer_base = deg2rad(25);
          steer_value /= (MPC::kMaxSteerAngle * MPC::kLf);

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = control_actuation.x_points;
          vector<double> mpc_y_vals = control_actuation.y_points;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
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
