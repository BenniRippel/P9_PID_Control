# CarND-Controls-PID
Udacity Self-Driving Car Engineer Nanodegree Program

## Project Introduction

In this project I implemented a PID Controller to steer a car in a simulator. The controller was tuned using a 
coordinate ascend algorithm.

## PID Control

A PID (proportional–integral–derivative) controller is a mechanism that continuously calculates
an error, which is the difference between a desired value (lateral position of the car i the middle of the road) and a 
measured value (actual lateral position). The controller uses this error to
calculate a correction value to steer an actuator (here: steering wheel). As a result, the error should decrease over time.

The correction value of a PID Controller or the time t is calculated by:

```
err[t] = desired_value[t] - measured_value[t]
sum_err[t] = sum(err[0:t])
diff_err[t] = err[t]-err[t-1]

correction[t] = Kp*err[t] + Ki*sum_err[t] + Kd*diff_err[t]
```

with the proportional gain Kp, the integral gain Ki, the differential gain Kd, the error for time t err[t],
the sum of all errors from time 0 to t sum_err[t] and the difference of the last errors diff_err[t].

#### P Value
The proportional part of the controller directly multiplies the error with the proportional gain Kp.
This leads to a fast response to errors but also to overshooting due to system dynamics and discrete measurements.
 

#### I Value
The integral part uses the sum of all previous errors multiplied with the integral gain, and is used to compensate biases.
Its contribution to the correction value gets bigger with longer lasting errors due to its integral nature.
 

#### D Value

The derivative part is proportional to the error's change over time. This leads to very fast reaction (high correction value) times at the moment 
when an error occurs, but to no correction for static errors.


## Parameter Tuning

A [coordinate ascend](https://en.wikipedia.org/wiki/Coordinate_descent) algorithm (presented in the lectures) was used to tune the parameters.
For starting parameters, I chose arbitrary values that didn't steer the car of track within the first few seconds. This decreases the 
 required 'training time'.
 
 The parameters are tuned by repeatedly running the simulator for a maximum of 500 timesteps 
 and analyzing the resulting error. A lower number of timesteps indicates that the car 
 left the track, which causes it to reset but still allows parameter tuning.
 
  
 The steering error for the coordinate ascend algorithm was calculated as follows:
 
``
error = sum(cte**2) / n**2
``

with the cross track error cte and the number of iterations (timestep) n.
Normalizing cte² by n² results in a small error for large n. This implies an indirect penalty 
for leaving the track.
The first 100 values of the recorded error are discarded to neglect the effect of overshooting in the beginning.


 
 



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

