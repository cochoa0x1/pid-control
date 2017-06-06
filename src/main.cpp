#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  //pid.Init(.1,0.1,0.001);


  double c[3] ={.2,.1,.1};

  if(argc ==4){
    std::cout << argv[1] << std::endl;
    c[0] = atof(argv[1]);
    c[1] = atof(argv[2]);
    c[2] = atof(argv[3]);
  }

  pid.Init(c[0],c[1],c[2]);

  /* params to decide when to terminate simulator */
  int t = 0 ; //keep track of the number of frames
  double terminal_cte = 1e7;
  int terminal_t = 1e7;

  if(argc ==6){
    terminal_t = atoi(argv[4]);
    terminal_cte = atof(argv[5]);
  }
  h.onMessage([&pid, &t, &terminal_cte, &terminal_t](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if(!pid.is_initialized){
      pid.Restart(ws);
    }

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      t++;
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        //std::cout << j << std::endl;

        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double throttle = std::stod(j[1]["throttle"].get<std::string>());
          double steer_value;
          double new_throttle;

          pid.UpdateError(cte);


          if(t >= terminal_t || ( cte > terminal_cte && t > 30) ){
            std::exit(0);
          }

          //std::cout << "angle: " << new_controls.steering_angle_;
          double q = .5;
          steer_value = q*pid.TotalError()+(1-q)*angle/25.0;
          new_throttle = .5*(1.-1.0*std::abs(steer_value));
          // DEBUG

          std::cout << "cte: " << cte
                    << " speed: " << speed
                    << " angle: " << angle
                    << " throttle: " << throttle
                    << " new_angle: " << steer_value
                    << " new_throttle: " << new_throttle
                    << " pe: " << pid.p_error
                    << " de: " << pid.d_error
                    << " ie: " << pid.i_error
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] =  new_throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;

  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
