#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_= Kp;
  Ki_=Ki;
  Kd_=Kd;
  integrated_cte =0.0;
}

Telemetry PID::updateTelemetry(Telemetry reading){

  //output packet
  Telemetry result;

  //handle case where there is no previous reading
  if (telemetry_.size() ==0){
    telemetry_.push_back(reading);
    result = reading;
    integrated_cte+=reading.cte;
    result.steering_angle_ = -1.0*Kp_*reading.cte - 1.0*Ki_*integrated_cte;
    return result;
  }

  //get the prior reading
  Telemetry last_reading = telemetry_.back();

  //save the new reading to our list
  telemetry_.push_back(reading);

  //calc the time since last reading
  double dt = (reading.timestamp_ - last_reading.timestamp_) / 1000.0;

  //calculate the cte derivative
  if(dt < .0000001){
    dt = .0000001; //guard against divide by zero somehow
  }

  double dcte = (reading.cte - last_reading.cte)/(dt);
  integrated_cte+=reading.cte*dt;

  result = reading;
  result.steering_angle_ = -1.0*Kp_*reading.cte -1.0*Kd_*dcte - 1.0*Ki_*integrated_cte;

  std::cout << "dt: " << dt << " cte: " << reading.cte << " dcte: " << dcte << " icte: " << integrated_cte << std::endl;
  return result;
}
