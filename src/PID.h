#ifndef PID_H
#define PID_H

#include <iostream>
#include <vector>
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

  double Kp_;
  double Ki_;
  double Kd_;

  std::vector<Telemetry> telemetry_; //list of all telemetry readings

  double integrated_cte; //running sum integrated cte

  PID();
  virtual ~PID();

  void Init(double Kp, double Ki, double Kd);

  Telemetry updateTelemetry(Telemetry reading);
};

#endif /* PID_H */
