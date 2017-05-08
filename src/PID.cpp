#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_= Kp;
  Ki_=Ki;
  Kd_=Kd;
  p_error = 0;
  i_error = 0;
  d_error = 0;
  last_cte =0;
  last_dcte=0;
  icte=0;
  is_initialized = false;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

void PID::UpdateError(double cte){

  // get system time in microseconds
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

  //handle case where there is no previous reading
  //disregard cte at time 0 for integrated term, do not want massive spike
  if (!is_initialized){
    //just update the p_error term, leave the rest 0
    p_error = -1.0*Kp_*cte;

    //set last_cte to this cte
    last_timestamp = timestamp;
    last_cte = cte;
    is_initialized = true;
    return;
  }

  //calc the time since last reading
  double dt = (timestamp - last_timestamp) / 1000000.0;
  if(dt < .0000001){ dt = .0000001; } //guard against divide by zero somehow

  icte += cte*dt; //update i term

  double q = .7;

  double dcte = q*(cte - last_cte)/(dt)+(1-q)*last_dcte;

  p_error = -1.0*Kp_*cte;
  i_error = -1.0*Ki_*icte;
  d_error = -1.0*Kd_*dcte;

  //updates for next loop
  last_timestamp = timestamp;
  last_cte = cte;
  last_dcte = dcte;
}

double PID::TotalError(){
  double err = p_error+i_error+d_error;
  // clamp the values to [-1,1]
  if(err >1.0){ err=1.0;}
  if(err <-1.0){ err=-1.0;}

  return err;
}
