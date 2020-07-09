#include <sstream> 
#include <iomanip>
#include <numeric>
#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_,
               unsigned filter_len_, double i_limit_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  err_prev = 0.0;
  i_limit = i_limit_/Ki;

  filter_len = filter_len_;
  if(filter_len > 1) {
    filter_inx = 0;
    filter_queue.resize(filter_len, 0.0);
  }
}

double PID::Update(double err) {
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
  
  err_prev = err_filt;

  return p_error + d_error + i_error;
}

double PID::Update(double err_, double dt_) {
  double err;
  
  if(filter_len > 1) {
    filter_queue[filter_inx++] = err_;
    filter_inx %= filter_len;
    err = std::accumulate(filter_queue.begin(), filter_queue.end(), 0.0)
        / filter_len;
  } else {
    err = err_;
  }

  p_error = err;

  if(dt_ < 0.0001) {  // div 0 protection
    d_error = 0.0;
  } else {
    d_error = (err - err_prev) / dt_;
  }

  i_error += dt_ * err;
  // anti-windup: clamp the integral
  if(i_error < -i_limit) {
    i_error = -i_limit;
  }
  if(i_error > i_limit) {
    i_error = i_limit;
  }

  err_prev = err;

  return (Kp * err) + (Ki * i_error) + (Kd * d_error);
}

void PID::Reset() {
  i_error = 0.0;
  err_prev = 0.0;
  filter_inx = 0;
  std::fill(filter_queue.begin(),filter_queue.end(), 0.0);
}


void PID::TwiddleInit(double dKp_, double dKi_, double dKd_,
                      double best_error_, double tolerance_) {
  dKp = dKp_;
  dKi = dKi_;
  dKd = dKd_;
  best_error = best_error_;
  
  twiddle_tolerance = tolerance_;

  twiddle_iteration = 0;

  i = 0;
  p[i] += dp[i];  // prepare for first iteration
  twiddle_state = TwiddleState::WAIT_RUN_A;

  twiddle_initialized = true;  
}

bool PID::TwiddleAdvance(double error_) {
  bool twiddle_done = false;

  switch(twiddle_state) {
    case TwiddleState::WAIT_RUN_A:
      if(error_ < best_error) {
        best_error = error_;
        dp[i] *= 1.1;
        if((i == p.size()-1) && (dpSum() < twiddle_tolerance)) {
          twiddle_done = true;
        }
        i = (i + 1) % p.size();
        if(i == 0) twiddle_iteration++;
        p[i] += dp[i];
        // re-enter WAIT_RUN_A
      } else {
        p[i] -= 2 * dp[i];
        twiddle_state = TwiddleState::WAIT_RUN_B;
      }
    break;

    case TwiddleState::WAIT_RUN_B:
      if(error_ < best_error) {
        best_error = error_;
        dp[i] *= 1.1;
      } else {
        p[i] += dp[i];
        dp[i] *= 0.9;
      }
      if((i == p.size()-1) && (dpSum() < twiddle_tolerance)) {
        twiddle_done = true;
      }
      i = (i + 1) % p.size();
      if(i == 0) twiddle_iteration++;
      p[i] += dp[i];
      twiddle_state = TwiddleState::WAIT_RUN_A;
    break;
  } // end switch(twiddle_state)

  return twiddle_done;
}


std::string PID::debug_string() {
  std::stringstream str;
  str << std::setprecision(5) << std::fixed << std::showpoint;
  str << "P, " << std::setw(8) << p_error * Kp << ",  "
      << "I, " << std::setw(8) << i_error * Ki << ",  "
      << "D, " << std::setw(8) << d_error * Kd << ",  "
      << std::ends;
  return str.str();
}