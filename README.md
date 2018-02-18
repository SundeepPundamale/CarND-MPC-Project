# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.


[image1]: ./data/KinematicModel.png "Kinematic Model"
## Rubric Points

* ### The Model

The MPC project applies a Kinematic model learned in lesson 19. The Kinematic model ignores the certain elements such as gravity, mass and tire forces. The model is based on the equations listed below:

![alt text][image1]

x,y,ψ,v are the states of the vehicle with x and y are the coordinates, ψ is the orientation angle and v is the velocity. δ is the steering angle, a is the acceleration and both act as actuator and control input to the system. cte is the cross track error which is the error between the center of the road and the vehicle's position and eψ is the orientation error. Based on the state and actuation from the previous timestamps the model calculates the current state.


* ### Timestep Length and Elapsed Duration (N & dt)
Tried various values for N & dt. Started with 25 and 0.05 as listed in lesson 20. The car went out of the track within no time. In addition tried other values such as 50 and 0.1, 5 and 0.025 but none of them helped. Eventually tried 10 and 0.1 as advocated in the [Udacity Q&A session](https://www.youtube.com/watch?v=bOQuhpz3YfU&list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2&index=5&t=0s)

N is the time stamp and dt is the duration. A large N has an impact on the cost function and a very small dt ends up running more steps and makes it computationally difficult and can slow down the execution.

* ### Polynomial Fitting and MPC Preprocessing
This is illustrated in main.cpp from lines 87 to 112. There are 6 waypoints which are received from the simulator. In the first step the x and y coordinates are initialised to zero and next step  psi (the orientation angle) is initialised to zero by rotating the angle. 

* ### Model Predictive Control with Latency
Latency is implemented in MPC.cpp from line 107 to 110. The kinematic equation depends on the state and actuation timesteps as described in the first rubric point above with a delay based on the timestep interval. The equation is modified such that the actuations are applied one timestep later. 


## Simulation

[Simulation result video](https://youtu.be/mrBiuTXhti0)

