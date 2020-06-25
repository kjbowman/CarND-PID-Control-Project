#ifndef PID_H
#define PID_H

#include <vector>
#include <string>


class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, unsigned filter_len_,
              double i_limit_, bool reset_i_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double err);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double ControlOutput();


  std::string debug_string();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Twiddle deltas
   */
  double dKp;
  double dKi;
  double dKd;

  /**
   * utility variables
   */
  double err_prev;
  double i_limit;
  std::vector<double> filter_queue;
  unsigned filter_len;
  int filter_inx;
  bool reset_i;
};

#endif  // PID_H