# MPC Control
Udacity Self-Driving Car Nanodegree Term 2, Project 5

## Reflection
### The Model
My MPC model starts out by taking in certain information from the simulator: 
* ptsx (x-position of waypoints ahead on the track in global coordinates)
* ptsy (y-position of waypoints ahead on the track in global coordinates)
* px (current x-position of the vehicle's position in global coordinates)
* py (current y-position of the vehicle's position in global coordinates)
* psi (current orientation angle of the vehicle, converted from the simulator's format to that expected in mathematical formulas)
* v (current velocity of the vehicle)
* delta (current steering angle of the car, i.e. where the wheels are turned, as opposed to the actual orientation of the car in the simulator at that point [psi])
* a (current throttle)

the update equations for update state is at 124-130 lines of "MPC.cpp"

### Polynomial Fitting & Preprocessing

I transform the points from the simulator's global coordinates into the vehicle's coordinates. This is done in lines 102-107 of `main.cpp`. First, each of the waypoints are adjusted by subtracting out px and py accordingly such that they are based on the vehicle's position. Next, the waypoint coordinates are changed using standard 2d vector transformation equations to be in vehicle coordinates:
* ptsx_car[i] = x * cos(-psi) - y * sin(-psi)
* ptsy_car[i] = x * sin(-psi) + y * cos(-psi)

### Tuning Timesteps (N) and Timestep Duration (dt) in MPC.cpp
I originally was using values of 15 for N and 0.2 for dt, because I thought 3 second (15 x 0.2 seconds) would be a good prediction span, and I also had thought I could account for latency in this way. However, I found 0.2 to be way too slow to react, plus I began accounting for latency in the `main.cpp` file. I also found the model seemed to slow down if N was higher, so I eventually settled on 10 for N, which meant that with 0.1 dt, I was only predicting for one second essentially. Given that the car can reach speeds of nearly 100 mph without any extremely erratic driving, this looks to be a great final spot for N and dt.


### Accounting for Latency
my model accounts for the simulator's added 100ms latency between the actuator calculation (when the model tells the car to perform a steering or acceleration/braking change) and when the simulator will actually perform that action.

I added in a step to predict where the vehicle would be after 100ms (0.1 seconds), in order to take the action that needed to actually be taken at that time, instead of the one in reaction to an old situation. I set the "dt" value here to equal the latency. Then, using the same update equations as those used in the actual MPC model, I predicted the state and fed that into the true model. This new predicted state, along with the coefficients, are then fed into the `mpc.Solve()` function found in `MPC.cpp`.

---

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
