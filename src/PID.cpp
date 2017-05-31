#include <cmath>
#include <algorithm>
#include "PID.h"


void PID::resetError(){
    p_error = 0;
    i_error += 0;
    d_error = 0;
    last_cte = 0;
}

void PID::UpdateError(double cte) {

    p_error = cte;

    i_error += cte;
    d_error = cte-last_cte;
    last_cte = cte;
}

void PID::TotalError() {
    double steer = -params[0]*p_error - params[1]*i_error - params[2]*d_error;
   // clip values to [-1.0, 1.0]
    steering= std::max(-1.0, std::min(1.0, steer));
}

void PID::reset_simulator(uWS::WebSocket<uWS::SERVER>& ws){
    // reset
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void PID::setAccError(double err){
    acc_err = err;
}

void PID::runTwiddle(){

    // error initialized?
    if(!err_init){ // init error and add first dp
        best_err = acc_err;
        best_params = params;
        err_init=true;
    }
    else {

        // check if twiddle thresh was passed, if so set twiddle to false
        double sum_dp = dp[0]+dp[1]+dp[2];
        if (sum_dp > thresh) {
            // adjust best_err and dp dependend on current op and current par
            if (acc_err < best_err){ // if new err is new best
                best_params = params;
                // set new err and adjust current dp *= 1.1
                best_err = acc_err;
                dp[current_par] *= 1.1;
                //  set next parameter, next ops is add, add dp to next parameter
                current_par = (current_par<2) ? (current_par+1):0;
                current_ops = 0;
                params[current_par] += dp[current_par];
                }

            else { // error didn't decrease
                if (current_ops==0) {// if adding didn't help, subtract
                    // subtract from current par
                    params[current_par] -= 2*dp[current_par];
                    // set current ops to subtracting
                    current_ops=1;

                }
                else { //if subtracting didnt help
                    // add dp again to current par
                    params[current_par] += dp[current_par];
                    // reduce dp
                    dp[current_par] *= 0.9;
                    // adjust current ops to add and par to next par in cycle
                    current_par = (current_par<2) ? (current_par+1):0;
                    current_ops = 0;
                    // add dp for next check
                    params[current_par] += dp[current_par];
                }
            }
        }
        else if (sum_dp <= thresh){
            std::cout<<"Disabling Twiddle due to sum_dp="<<sum_dp<< " <= thresh!"<< std::endl;
            twiddle = false;
            std::cout<<"Parameters found by twiddling"<<std::endl;
            std::cout<<"P: "<<best_params[0]<<std::endl;
            std::cout<<"I: "<<best_params[1]<<std::endl;
            std::cout<<"D: "<<best_params[2]<<std::endl;

        }
    }
}