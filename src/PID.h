#ifndef PID_H
#define PID_H

#include <iostream>
#include <vector>
#include <uWS/uWS.h>
#include <ctime>
#include <chrono>


class PID {
public:

  double p_error;
  double i_error;
  double d_error;

  /* Coefficients */
  double Kp_;
  double Ki_;
  double Kd_;

  long long last_timestamp;
  double last_cte;
  double last_dcte; //used for smoothing
  double icte;
  bool is_initialized;

  PID();
  virtual ~PID();

  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /* restart the simulator */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);
  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
