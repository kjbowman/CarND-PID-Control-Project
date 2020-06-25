#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
// #include <chrono>
#include <ctime>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::min;
using std::max;
using std::cout;


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
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


void RestartSimulator(uWS::WebSocket<uWS::SERVER>ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


int main() {
  uWS::Hub h;


  bool initialized = false;

  long step_count = 0L;
  
  auto t_prev = std::chrono::high_resolution_clock::now();
  // double t_prev = clock();

  // lane centering control
  PID center_pid;
  // center_pid.Init(0.10, 0.0007, 3.3, 5, 0.7, false); // good for 20 mph
  // center_pid.Init(0.10, 0.00105, 4.9, 5, 1.0, false); // good for 30 mph
  // center_pid.Init(0.08, 0.0008, 4.9, 10, 1.0, false); // good for 30 mph ?
  // center_pid.Init(0.10, 0.00157, 3.5, 5, 0.7, false);
  // 30 mph
  // center_pid.Init(0.8, 0.02, 40, 5, 25.0, false); // good for 30 mph
  center_pid.Init(1, 0.05, 20, 5, 25.0, false); // good for 30 mph

  // Steering set point control
  PID sp_pid;
  // sp_pid.Init(0.16, 0.355, 0.055, 0, 1.0, false);  //Kp = 0.16, Ki = 0.355, Kd = 0.055, no filtering
  sp_pid.Init(0.0064, 0.0142, 0.0022, 0, 1.0, false);  //Kp = 0.16, Ki = 0.355, Kd = 0.055, no filtering

  // throttle control
  PID v_pid;
  v_pid.Init(0.5, 0.003, 2.0, 15, 1.0, false);  
  double set_speed = 30.0;

  PID th_pid;
  th_pid.Init(0.3, 0.5, 0, 0, 1.0, false);  // 0.3, 0.5 0 for fast, even response

  h.onMessage([&initialized, &step_count, &center_pid, &sp_pid, &v_pid, &th_pid, &set_speed, &t_prev]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double throttle = std::stod(j[1]["throttle"].get<string>());
          // auto image = j[i]["image"];  // what format?

          if(!initialized) {
            t_prev = std::chrono::high_resolution_clock::now();
            initialized = true;
          }

          double steer_value;
          double set_point;
          double err;
          double throttle_value;

          auto t_now = std::chrono::high_resolution_clock::now();
          // double t_now = clock();
          std::chrono::duration<double> t_diff = t_now - t_prev;
          double dt = t_diff.count();
          // double delta_t = (t_now - t_prev) / CLOCKS_PER_SEC;
          t_prev = t_now;

          step_count++;
          // if(step_count > 100) {
          //   RestartSimulator(ws);
          //   step_count = 0;
          // }

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          err = -cte;
          center_pid.UpdateError(err);
          set_point = center_pid.ControlOutput();
          if(set_point > 25.0) {
            set_point = 25.0;
          }
          if(set_point < -25.0) {
            set_point = -25.0;
          }

          err = set_point - angle;
          sp_pid.UpdateError(err);
          steer_value = sp_pid.ControlOutput();
          if(steer_value > 1.0) {
            steer_value = 1.0;
          }
          if(steer_value < -1.0) {
            steer_value = -1.0;
          }

          err = set_speed - speed;
          v_pid.UpdateError(err);
          double throttle_setpt = v_pid.ControlOutput();
          if(throttle_setpt > 1.0) {
            throttle_setpt = 1.0;
          }
          if(throttle_setpt < 0.0) {
            throttle_setpt = 0.0;
          }

          err = throttle_setpt - throttle;
          th_pid.UpdateError(err);
          throttle_value = th_pid.ControlOutput();
          if(throttle_value > 1.0) {
            throttle_value = 1.0;
          }
          if(throttle_value < 0.0) {
            throttle_value = 0.0;
          }

          // DEBUG
          cout << std::setprecision(5)
               << std::fixed
               << std::showpoint;
          cout << "step, "         << std::setw(7) << step_count     << ", "
               << "dt, "           << std::setw(9) << dt             << ", "
               << "cte, "          << std::setw(9) << cte            << ", "
               << "steer angle, "  << std::setw(9) << angle          << ", "
              //  << "speed, "        << std::setw(9) << speed          << ", "
              //  << "throttle, "     << std::setw(9) << throttle       << ", "
               << "steer setpt, "  << std::setw(9) << set_point*25   << ", "
               << "steer cmd, "    << std::setw(9) << steer_value*25 << ", "
              //  << "throttle setpt, "  << std::setw(9) << throttle_setpt   << ", "
              //  << "throttle cmd, " << std::setw(9) << throttle_value
               << center_pid.debug_string()
               << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;  // 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      } // end s != "" (hasData)
    }  // end websocket message if
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