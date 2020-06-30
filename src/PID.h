#ifndef PID_H
#define PID_H

#include <vector>
#include <functional>
#include <string>

enum class TwiddleState {WAIT_RUN_A, WAIT_RUN_B};

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   * @param filter_len_ size of moving average filter for error
   *                      - set to 0 or 1 for no filtering
   * @param i_limit_ saturation value for integrator (anti-windup)
   */
  void Init(double Kp_, double Ki_, double Kd_,
            unsigned filter_len_, double i_limit_);

  /**
   * Initialize the Twiddler
   * @param (dKp_, dKi_, dKd_)
   * @param best_error_ the sum of squares error from the first run
   * @param tolerance_ tiddling will end when sum(dp) < tolerance
   */
  void TwiddleInit(double dKp_, double dKi_, double dKd_,
                   double best_error_, double tolerance_);

  /**
   * Advance the Twiddler state machine
   * @param error_
   */
  bool TwiddleAdvance(double error_);

  /**
   * return the sum of the Twiddler paremter deltas
   */
  double dpSum() { return dKp + dKi + dKd; }

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double err);

  /**
   * Reset the integrator and other "history"
   */
  void Reset();

  /**
   * Calculate the total PID error.
   */
  double ControlOutput();

  /**
   * formats the current p, i, & d errors into a string for printing
   */
  std::string debug_string();

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Twiddle parameters
   */
  bool twiddle_initialized {false};
  double dKp;
  double dKi;
  double dKd;
  double best_error;
  std::vector<std::reference_wrapper<double>> p {Kp, Ki, Kd};
  std::vector<std::reference_wrapper<double>> dp {dKp, dKi, dKd};
  unsigned i;  // cycle through gain parameters
  unsigned twiddle_iteration;
  TwiddleState twiddle_state;
  double twiddle_tolerance;

 private:

  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * utility variables
   */
  double err_prev;
  double i_limit;
  std::vector<double> filter_queue;
  unsigned filter_len;
  int filter_inx;
};

#endif  // PID_H