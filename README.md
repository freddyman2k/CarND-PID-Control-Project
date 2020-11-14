# CarND-Controls-PID
A C++ Implementation of a simple PID Controller. 

Given the current cross track error, the controller determines the steering angle of the vehicle, so that it quickly and smoothly returns to the center of the lane. Tested using the Term 2 Simulator provided by Udacity as part of the [Self Driving Car Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).

![](pid_full_lap.mp4)

---

## How does it work?
A proportional–integral–derivative (PID) controller has three terms:
* The _proportional_ (**P**) term increases or decreases proportionally with the lateral displacement (_cross track error_, **cte**) of the vehicle. This means that if the vehicle is far from its desired position at the center of the lane, the P term is large, causing the vehicle to steer back to the center.
* An _integral_ term (**I**) is used to compensate a _steady-state error_ (or _lane offset_) in cases where the vehicle's nominal performance changes due to environmental factors. (Imagine a long truck on a bridge, where the driver needs to adjust his steering because of strong winds pushing the truck aside.)
* Lastly, a _derivative_ term (**D**) is added, measuring the rate at which the cross track error changes. This prevents the car from moving too quickly in the direction of the lane center, which would cause the car to "overshoot" onto the other side of the lane. 

Properly choosing these parameters is the main task in constructing a good controller. [This video](https://www.youtube.com/watch?v=4Y7zG48uHRo&feature=emb_title) nicely illustrates the effects of increasing/decreasing the P, I and D terms.

With the simulation, one could also use an algorithm like [SGD](https://en.wikipedia.org/wiki/Stochastic_gradient_descent) to automatically find the parameters that optimize the vehicle's performance. In this project, I did not make use of automated parameter optimization and manually optimized the parameters instead.

To further optimize the car's lap time, one could also use a second controller for the speed. This was not implemented here, the car simply drives at a constant throttle value (`0.3`), with a maximum speed of 15 MPH.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
* Simulator. You can download these from [the Udacity repo](https://github.com/udacity/self-driving-car-sim/releases).

While it is recommended to use Ubuntu (e.g. with [WSL](https://docs.microsoft.com/en-us/windows/wsl/install-win10)), fellow udacity students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf). There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

After that, run the simulator and select "Project 4: PID Controller". Your car should now follow the road!

## Next steps
In the future, it might be interesting to explore the following ideas:
* Running the simulation multiple times and automatically finding optimized parameters using techniques such as
  * Twiddle
  * Stochastic Gradient Descent
  * ...
* Trying to minimize the time it takes the car to drive one lap by using a second controller for the speed (throttle)
* Implementing and comparing other controllers (e.g. Stanley, MPC)