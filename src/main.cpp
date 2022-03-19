#include "PID.h"
#include "json.hpp"
#include <iostream>
#include <math.h>
#include <string>
#include <uWS/uWS.h>

// for convenience
double clamp(double steer_value, double min, double max);
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID steering_pid;
  PID speed_pid;
  /**
   * TODO: Initialize the pid variable.
   * DONE
   */
  steering_pid.Init(1.5, 0.004, 10.0);
  speed_pid.Init(1.0, 0.004, 2.0);
//  speed_pid.Init(2.0, 0.004, 2.0);

  const double MAX_SPEED = 50;
  const double MAX_STEERING_ANGLE = 25;
  const double FULL_STEER_MAX_SPEED = 20;

  h.onMessage([&steering_pid, &speed_pid, &MAX_SPEED, &MAX_STEERING_ANGLE,
               &FULL_STEER_MAX_SPEED](uWS::WebSocket<uWS::SERVER> ws,
                                      char *data, size_t length,
                                      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;


          /* Steering */

          // Principle - steer in proportion to cte
          // Principle - steer more gently at high speeds

          steering_pid.UpdateError(cte );
          steer_value = steering_pid.TotalError() / (speed/4);
          steer_value = tanh(steer_value) / tanh(1);
          steer_value = clamp(steer_value, -1, 1);

          /* Speed */

          // Principle - try to reach the max speed
          // Principle - reduce target speed when sharp steering is applied.

          // Calculate a non-linear drop in speed to apply during sharp steering
          // target_speed peaks at MAX_SPEED and goes down with rate of steering
          // The exp formulation drops the speed quickly on small turns.
          // TOOD: Consider an alternative where slow down is slow for small
          // angles and high speeds
          //          double target_speed =
          //          exp(-(0.03*angle_percent-log(MAX_SPEED-FULL_STEER_MAX_SPEED)))+FULL_STEER_MAX_SPEED;


          // Linear slowdown proportional to steering angle
//          double angle_percent = abs(angle / MAX_STEERING_ANGLE) * 100;
//          double target_speed = MAX_SPEED - (MAX_SPEED - FULL_STEER_MAX_SPEED) /
//                                                MAX_SPEED * angle_percent;

          double slowdown_on_steer = (MAX_SPEED - FULL_STEER_MAX_SPEED) * abs(steer_value);
          double target_speed = MAX_SPEED - slowdown_on_steer;

          double speed_error = target_speed - speed;
          speed_pid.UpdateError(speed_error);
          double throttle = -1 * speed_pid.TotalError();
          throttle = tanh(throttle/100) / tanh(1);
          throttle = clamp(throttle, -1, 1);
//          double throttle = 0.3;
//          double target_speed = MAX_SPEED;


          printf("%.17f,%.17f,%.17f,%.17f,%.17f,%.17f\n", cte, angle, speed, steer_value, target_speed, throttle);

//          // DEBUG
//          std::cout << "CTE: " << cte << " angle: " << angle
//                    << " -- steering: " << steer_value << " Speed: " << speed
//                    << " target: " << target_speed << " -- throttle: "
//                    << throttle
//                    //                    << " Speed: " << speed << " target: "
//                    //                    << target_speed << " turn_slowdown: "
//                    //                    << speed_down_on_turn << " --
//                    //                    throttle: " << throttle
//                    << std::endl;


          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

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


double clamp(double val, double min = -1, double max = 1) {
  if (val > max)
    val = max;
  if (val < min)
    val = min;
  return val;
}
