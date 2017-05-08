#ifndef PID_H
#define PID_H

#include <iostream>
#include <vector>
#include <uWS/uWS.h>
/**
  * structure to hold telemtry data
**/
struct Telemetry {
	long long timestamp_;
	double steering_angle_;
  double velocity_;
  double throttle_;
  double cte;
};


class PID {
public:

  /* Coefficients */
  double Kp_;
  double Ki_;
  double Kd_;

  bool is_initialized;

  std::vector<Telemetry> telemetry_; //list of all telemetry readings

  double integrated_cte; //running sum integrated cte

  PID();
  virtual ~PID();

  void Init(double Kp, double Ki, double Kd);

  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  Telemetry updateTelemetry(Telemetry reading);
  double avg_cte();
};

#endif /* PID_H */
