#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "PID.h"


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

int main()
{
    PID pid = {0.1187, 0.00033, 1.3385};
    pid.twiddle=false; // set twiddle to true if you want to tune parameters

    uWS::Hub h;
    int n = 0;
    double err=0;
    h.onMessage([&pid, &n, &err](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event

        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value;

                    n +=1;

                    // twiddle if pid.twiddle == true
                    if (pid.twiddle) {
                        //acc error for n>100
                        if (n > 100) { err += cte * cte;}

                        // Twiddle in all following cases
                        if ((n > 500) || (std::fabs(cte) > 3.0)) {
                            pid.reset_simulator(ws);
                            pid.resetError();
                            // twiddle parameters
                            err = err/(n*n);

                            std::cout << "Error: " << err <<" Best Error so far: "<<pid.best_err << std::endl;
                            std::cout << "Best Parameters so far" << std::endl;
                            std::cout << "P: " << pid.best_params[0] << "  I: " << pid.best_params[1] << "  D: "
                                      << pid.best_params[2] << std::endl;
                            std::cout<<"_______________________________"<<std::endl;
                            pid.setAccError(err);
                            pid.runTwiddle();

                            // skip running the parameters if any parameter is negative (doesn't make sense)
                            if ((pid.params[0]<0)||(pid.params[1]<0)||(pid.params[2]<0)){
                                pid.setAccError(1000000);
                                pid.runTwiddle();
                            }
                            std::cout << "Parameters found by twiddling" << std::endl;
                            std::cout << "P: " << pid.params[0] << "  I: " << pid.params[1] << "  D: "
                                      << pid.params[2] << std::endl;
                            std::cout << "dP: " << pid.dp[0] << "  dI: " << pid.dp[1] << "  dD: " << pid.dp[2]<< std::endl;
                            // reset n and err
                            n = 0;
                            err = 0;
                        }
                    }

                    /*
                    * [-1, 1].
                    * NOTE: Feel free to play around with the throttle and speed. Maybe use
                    * another PID controller to control the speed!
                    */
                    pid.UpdateError(cte);
                    pid.TotalError();
                    steer_value = pid.steering;

                    // DEBUG
                    //std::cout<< "Angle from Sim: "<< angle << std::endl;
                    //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = std::max(0.0, std::min(0.1*(50.0-speed), 1.0));
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
    }
    h.run();
    std::cout<< n <<std::endl;
  return 0;

}


