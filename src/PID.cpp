#include <cmath> 
#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients and errors
   */
  this->params[0] = Kp_;
  this->params[1] = Ki_;
  this->params[2] = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  d_error = cte - previous_cte;
  i_error += cte;
  previous_cte = cte;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  double total_error = -params[0] * previous_cte - params[2] * d_error - params[1] * i_error; // After the update step, previous_cte contains the current cte
  if (total_error > 1) total_error = 1.0;
  if (total_error < -1) total_error = -1.0;
  return total_error;
}
