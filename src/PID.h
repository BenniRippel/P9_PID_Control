#ifndef PID_H
#define PID_H

#include <uWS/WebSocketProtocol.h>
#include <uWS/WebSocket.h>
#include <uWS/Hub.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error = 0.0;
  double i_error= 0.0;
  double d_error = 0.0;
  double last_cte = 0.0;

  /*
  * Coefficients
  */ 
  std::vector<double> params;
  std::vector<double> best_params{0, 0, 0};

/*
 * Steering value
 */
  double steering;

 /*
  * Twiddle
  */
  bool twiddle = true;
  std::vector<double> dp{0.1, 0.0001, 0.1};
  double acc_err = 0;
  int current_par = 0;
  int current_ops = 0;
  double best_err;
  bool err_init = false;
  double thresh=0.005;


  /*
  * Constructor
  */
  PID(std::initializer_list<double> l): params(l) {};

  /*
  * Destructor.
  */
  virtual ~PID(){};



  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  void TotalError();

  void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws);

  void setAccError(double err);

  void resetError();

  void runTwiddle();

};

#endif /* PID_H */
