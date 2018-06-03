# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

In this project we'll go again around the lake race track from the Behavioral Cloning and PID controller projects. This time, we'll implement an MPC (Model Process Control) controller in C++ to maneuver the vehicle around the track!

## Objective

The objective of this project is to have the car race around the track safely (without leaving the track bounds) as fast as possible by controlling it with an MPC controller.

A successful implementation requires:

* Code that compiles correctly with CMake and Make
* The model used for MPC must be the Global Kinematic Model for a vehicle
* Select the appropriate values for the Number of timesteps (N) and Timestep duration (dt)
* Fit a polynomial curve to the waypoints provided by the simulator
* Deal at least with a 100ms latency between actuation and response
* Successfully complete at least a full lap around the track

[image1]: ./ModelEquations.png
[image2]: ./Simulation.gif

### Here I will consider each point individually and explain how I addressed them in my implementation

#### Compilation

The starter code provided for the project was already meant to compile correctly in Linux, Mac and Windows; although compilation in Windows required installing Windows 10 Bash on Ubuntu or using Docker to create a virtual environment.

In previous projects for this term, I had chosen to develop natively in Windows. This presented some issues, but after the first project of the term (Extended Kalman Filter) I had an environment that would allow me to build most other projects.

For the MPC-Quizzes and this project, however, the installation of one of the components in Windows required a lot of extra work and was not guaranteed to work. Due to time constraints, I chose to build the project in Linux, and this presented an additional challenge: the simulator would not run in my Linux environment, so I was running it in my Windows box and the MPC program in my Linux box. This added some real latency between actuation calculation and the application of such actuation.

There are a couple of changes in the CMakeLists file to include an additional source file and to copy the parameter configuration file from the source folder to the build folder. Otherwise, the build configuration remains the same.

#### Use the Global Kinematic Model for the MPC controller

The Global Kinematic Model is the simplified vehicle model where the car is considered to have the same constraints as a bicycle. While this model doesn't address a lot of the car's internal or external forces it is a good approximation (at low and moderate speeds) that can be calculated fast enough for appropriate control of the vehicle.

The choice of vehicle model is governed by the following equations:

![The equations of a vehicle's global kinematic model][image1]

What these equations describe is the following:
* The position of the car at the next timestep (x_t + 1 and y_t + 1) can be calculated from the current position (x_t, y_t) plus the speed (v_t) times the duration of the timestep (dt) times the component in the x and y coordinates of that speed, based on the heading (psi_t).
* The heading at the next timestep will be the current heading, plus the change in the heading (delta) times the speed times the duration of the timestep, all divided by the distance between the front axle of the vehicle (where the change in heading is applied) to the center of gravity (variable Lf). Using this last variable Lf recognizes in the model that the vehicle is not a simple point but has some length. This length affects the vehicle's turn rate, affecting the model in turn.
* The speed at the next time step will be the current speed plus the acceleration (a_t) times the duration of the timestep.

Of these variables, x_t, y_t, psi_t and v_t are variables that describe the current state of the vehicle as measured by the on-board sensors. A_t and delta are control variables, which we apply to modify the state of the vehicle. Lf is a characteristic of the vehicle that may change from one vehicle to another but never for the same vehicle. Lastly dt is a model variable, chosen to find the state of the next vehicle at a future moment in time.

The model's equations can be seen in the adjustment for latency (main.cpp:114-117) and the definition of the controller's constraints for the optimizer (MPC.cpp:164-169).

#### Appropriate values for N and dt

Although it has not yet been discussed, to control a vehicle we want to have good idea of its trajectory over time, not just a single point. This is, in the controller, where the variable N comes in: it defines how many timesteps into the future we want to calculate the trajectory for.

It is not practical to predict the trajectory of a car from its starting point to its destination in order to control things like heading and speed. From the computational point of view, trying to predict the whole trajectory would require massive resources to do so in a useful amount of time. From the practical point of view, changes in environmental conditions (other cars, differences between the expected and real speed and heading, etc.) will reduce the usefulness of such a prediction. Fast.

The idea then, is to predict a small number of timesteps into the future. This is the N variable.

The same environmental conditions somewhat dictate how far into the future we want our prediction to go (the duration of the timestep). Larger timestep durations will attempt to predict farther into the future. Shorter timestep durations will require more computations to track the vehicle's movements. From a safety point of view, the faster a vehicle moves, the further it travels with each timestep. This is why a human driver must be more vigilant as the speed increases. I think the same applies for cars.

This leads us to seemingly competing concerns: we want to be able to control the car pre-emptively based on the trajectory and we want to be able to respond to changes, all with a regular amount of computing power.

In general terms, the combination of N and dt will tell us how many seconds into the future we will be able to predict. For example, with 20 steps at 0.1 seconds per step, we are calculating the vehicle's trajectory two seconds into the future. These were actually my starting values, and they worked well for slower speeds in my environment ( < 40mph ). Anything over that, caused the trajectory optimization inside the controller to come up with adjustment values that were less than ideal and caused the vehicle to swerve or go off road.

My final values were N = 10 and dt = 0.1 (for a total of one second look-ahead). They are defined in MPC.cpp:14-15.

#### Polynomial fitting and MPC preprocessing

One of the first steps in using a MPC controller is the calculation of CTE (Cross Track Error, or the distance between the expected position and the actual position) and EPSI (the error between the expected and actual heading). In order to get these values, a curve must be fitted to the telemetry-calculated waypoints so that we may calculate the distance and the curve's tangent.

One caveat in the project is that the waypoints are provided in the map’s coordinate system. While not impossible to do the required calculations with the raw data, the process is simplified greatly if the points are expressed in the car's coordinate system. To do this, we apply a homogeneous transformation to translate and rotate the waypoints (main.cpp:73-81).

Once this is done, we fit a third degree polynomial to the waypoints. A third degree is used since it can be used to describe most curves found on roads. The fitting is done with a function borrowed from the Julia language's source (the github link can be found in helper_functions.h:27) and relies heavily on the Eigen library (main.cpp:92-95).

With the polynomial at hand, we calculate the expected position of the car by evaluating the polynomial at the current position of the car (x = 0 since we shifted the coordinate system). The CTE is the difference between the current y position (0 again) and the calculated one (main.cpp:99).

Similarly, we calculate the tangent of the curve, which is greatly simplified by the fact that x = 0 in car coordinates for the current position. With this value, we can get the error in the heading by subtracting the current heading (0 once more) and the calculated one (main.cpp:103-108).

#### MPC and dealing with latency

Our MPC is called once per telemetry event to solve the actuator controls for the current state and desired trajectory curve.

The controller is comprised of two parts, the first sets the constraint values on the variables passed to the optimizer. This is done in the main body of the Solve method of the MPC class (MPC.cpp:205-256). To feed into the solver, vectors are created to hold the necessary information. It is important to notice how the value of the first (of N, the number of timesteps to predict) variable entries and (upper and low-bound) constraints are set to the current state. This will set the stage for the solver's solution.

The second part is in the form of a function representation to optimize (FG_eval @ MPC.cpp:46). This class fills a vector of the cost constraints, both general (stored in index zero) and for the variables.

The general cost is a compilation of all the factors that we should take into account as objectives (low CTE and EPSI, close to a reference speed, don't accelerate, break or swerve too hard). Each of these terms has an importance assigned to it. In this project, the importance factor of each parameter, is loaded from a text file when the solver is instantiated (MPC.cpp:52-66).

The cost of the variables is derived from the model's equations: the predicted state at some step x+1 must be equal to the previous step x plus the model's estimation (MPC.cpp:164-169) so the latter is subtracted from the former to get the cost of the current estimation.

Once the actuation values are optimized (to minimize the cost) a structure (MPC.h:12-17) is used to handle the return values for acceleration and steering as well as the projected trajectory. This makes it easy to use the values once the solver returns.

Now, it is common for there to be a delay between the calculation of an actuation value, the application of such actuation and the actual response to the actuation. This delay will be called latency.

The project requires that the car at least handles 100ms latency, however, as explained before my environment was split between two computers (one running the code and one running the simulator). This led me to self-impose a 150ms latency to account for communication delays.

The way that latency is handled in the code is by passing an updated state to the MPC solver (main.cpp:113-117). This way we are attempting to calculate the actuators and trajectory for our projection of where the car will be instead of the current position of the car.

To do this, one important consideration is that the delta and acceleration variables have to be approximated from the values returned by the telemetry. In our case, we have the steering and acceleration actuator values (which go from -1 to 1). To go from steering value to delta, we multiply it times the maximum angle allowed (25Â°). To go from acceleration value to acceleration we multiply it times 2.5, an approximation of the maximum acceleration (the car in the simulator goes from 0 to 40 mph in 4 seconds).

#### Driving around the track

Here is a short animation of the vehicle driving around, with the debugging lines displayed.

![The car driving around the track controlled by an MPC][image2]

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

