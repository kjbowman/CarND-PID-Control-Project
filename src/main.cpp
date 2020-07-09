#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <chrono>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::min;
using std::max;
using std::cout;

// For Twiddling
constexpr bool TWIDDLE = false;
constexpr unsigned TW_N_SAMPLES = 210;
constexpr unsigned TW_SKIP_SAMPLES = 10;
constexpr unsigned TW_AVG_SAMPLES = TW_N_SAMPLES - TW_SKIP_SAMPLES;
constexpr double TW_TOLERANCE = 0.001;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 */
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

/**
 *  Send command to simulator to restart (as if from beginning)
 */
void RestartSimulator(uWS::WebSocket<uWS::SERVER>ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


int main() {
  uWS::Hub h;

  bool initialized = false;
  long step_count = 0L;
  auto t_start = std::chrono::high_resolution_clock::now();
  auto t_prev = t_start;

  // lane centering control
  PID center_pid;
  center_pid.Init(1.6, 1.4, 2.9, 5, 25.0);

  // Steering set point control
  PID sp_pid;
  sp_pid.Init(0.002, 0.1, 0.00005, 3, 1.0);

  // speed control
  PID v_pid;
  v_pid.Init(0.3, 0.4, 2.5, 10, 1.0);
  double set_speed = 25.0;    // How fast do you want to go?

  // throttle control
  PID th_pid;
  th_pid.Init(0.3, 2, 0.001, 0, 1.0);

  PID* tw_pid = &center_pid;  // pointer to which PID you want to twiddle

  double twiddle_err_sum = 0.0;

  h.onMessage([&initialized, &step_count,
               &center_pid, &sp_pid, &v_pid, &th_pid, &set_speed, &t_prev, &t_start,
               &tw_pid, &twiddle_err_sum]
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
            t_start = std::chrono::high_resolution_clock::now();
            t_prev = t_start;
            initialized = true;
          }

          double steer_value = 0.0;
          double set_point;
          double center_err;
          double err;
          double throttle_setpt;
          double throttle_value = 0.0;

          auto t_now = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> t_diff = t_now - t_prev;
          std::chrono::duration<double> t_elapsed = t_now - t_start;
          double dt = t_diff.count();
          double run_time = t_elapsed.count();
          t_prev = t_now;

          step_count++;

          // ----- TWIDDLER CODE -----
          if(TWIDDLE) {
            // set the appropriate error for the twiddled pid
            // err = set_speed - speed;    // for the speed control
            err = -cte;                    // for the lane centering control
            
            cout << "\rstep:\t" << std::setw(7) << step_count  << "\t" << std::flush;

            if(step_count > TW_SKIP_SAMPLES) {
              twiddle_err_sum += (err * err); // sum the square of the error
            }
            if(step_count > TW_N_SAMPLES) {
              if(!tw_pid->twiddle_initialized) {
                tw_pid->TwiddleInit(0.5, 0.5, 0.5,
                                  twiddle_err_sum/TW_AVG_SAMPLES, TW_TOLERANCE);
              } else {
                bool tw_done = tw_pid->TwiddleAdvance(twiddle_err_sum/TW_AVG_SAMPLES);
                if(!tw_done) {
                  cout << std::endl;
                  cout << "***** Twiddle iteration: " << tw_pid->twiddle_iteration << "  ";
                  cout << "Kp: " << tw_pid->Kp << "  "
                      << "Ki: " << tw_pid->Ki << "  "
                      << "Kd: " << tw_pid->Kd << "  "
                      << "err: " << twiddle_err_sum/TW_AVG_SAMPLES << std::endl;
                } else {
                  cout << std::endl
                      << std::endl;
                  cout << "*************** Done Twiddling ***************" << std::endl;
                  cout << std::endl;
                  cout << "Final parameters after " << tw_pid->twiddle_iteration << " iterations:" << std::endl;
                  cout << "Kp: " << tw_pid->Kp << "  "
                      << "Ki: " << tw_pid->Ki << "  "
                      << "Kd: " << tw_pid->Kd << std::endl;
                  cout << std::endl;
                  cout << "**********************************************" << std::endl;
                  exit(1);
                }
              }
              cout << std::endl;
              cout << "** Simulator Reset **" << std::endl;
              RestartSimulator(ws);
              center_pid.Reset();
              sp_pid.Reset();
              v_pid.Reset();
              th_pid.Reset();
              twiddle_err_sum = 0.0;
              step_count = 0;
              steer_value = 0.0;
              throttle_value = 0.0;
            }
          } // end if(TWIDDLE)

          if(step_count > 0) {  // if the simulation was just reset, ignore the first step
            err = -cte;
            center_err = err;
            set_point = center_pid.Update(err, dt);
            if(set_point > 25.0) {
              set_point = 25.0;
            }
            if(set_point < -25.0) {
              set_point = -25.0;
            }

            err = set_point - angle;
            steer_value = sp_pid.Update(err, dt);
            if(steer_value > 1.0) {
              steer_value = 1.0;
            }
            if(steer_value < -1.0) {
              steer_value = -1.0;
            }

            err = set_speed - speed;
            throttle_setpt = v_pid.Update(err, dt);
            if(throttle_setpt > 1.0) {
              throttle_setpt = 1.0;
            }
            if(throttle_setpt < 0.0) {
              throttle_setpt = 0.0;
            }

            err = throttle_setpt - throttle;
            throttle_value = th_pid.Update(err, dt);
            if(throttle_value > 1.0) {
              throttle_value = 1.0;
            }
            if(throttle_value < 0.0) {
              throttle_value = 0.0;
            }
          }

          // DEBUG
          if(!TWIDDLE) {  // we don't want to see this during Twiddling
            cout  << std::setprecision(5)
                  << std::fixed
                  << std::showpoint;
            cout  << "step, "         << std::setw(7) << step_count     << ", "
                  << "dt, "           << std::setw(9) << dt             << ", "
                  << "t, "            << std::setw(9) << run_time       << ", "
                  << "cte, "          << std::setw(9) << -center_err    << ", "
                  << "steer angle, "  << std::setw(9) << angle          << ", "
                  << "speed, "        << std::setw(9) << speed          << ", "
                  // << "throttle, "     << std::setw(9) << throttle       << ", "
                  << "steer setpt, "  << std::setw(9) << set_point      << ", "
                  << "steer cmd, "    << std::setw(9) << steer_value*25    << ", "
                  // << "throttle setpt, "  << std::setw(9) << throttle_setpt  << ", "
                  // << "throttle cmd, " << std::setw(9) << throttle_value  << ", "
                  << center_pid.debug_string()
                  << std::endl;
          }

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
