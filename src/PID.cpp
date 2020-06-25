#include <strstream>
#include <iomanip>
#include <numeric>
#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, unsigned filter_len_, 
                double i_limit_, bool reset_i_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  err_prev = 0.0;
  i_limit = i_limit_;

  filter_len = filter_len_;
  if(filter_len > 1) {
    filter_inx = 0;
    filter_queue.resize(filter_len, 0.0);
  }

  reset_i = reset_i_;
}

void PID::UpdateError(double err) {
  double err_filt;
  
  if(filter_len > 1) {
    filter_queue[filter_inx++] = err;
    filter_inx %= filter_len;
    err_filt = std::accumulate(filter_queue.begin(), filter_queue.end(), 0.0)
              / filter_len;
  } else {
    err_filt = err;
  }

  p_error = Kp * err_filt;
  d_error = Kd * (err_filt - err_prev); 
  i_error += Ki * err_filt;
  
  // anti-windup: clamp the integral
  if(i_error < -i_limit) {
    i_error = -i_limit;
  }
  if(i_error > i_limit) {
    i_error = i_limit;
  }
  
  // reset integral on zero crossing
  if(reset_i) {
    if(err_prev * err_filt < 0.0) {
      i_error = 0;
    }
  }

  err_prev = err_filt;
}

double PID::ControlOutput() {
  return p_error + d_error + i_error;
}

std::string PID::debug_string() {
  std::strstream str;
  str << std::setprecision(5) << std::fixed << std::showpoint;
  str << "P, " << std::setw(8) << p_error << ",  "
      << "I, " << std::setw(8) << i_error << ",  "
      << "D, " << std::setw(8) << d_error << ",  "
      << std::ends;
  return str.str();
}