[//]: # (Image References)
[state]:  ./images/state-example-1.png
[model]:  ./images/kinematic-model-formulas.png

## The Model

In this implementation I used the global kinematic model.  This model ignores some vehicle forces such as drag and tire forces along with gravity and mass.
This implementation is more simple compared to a dynamic model, but it is accurate enough for the vehicle in the simulation even traveling in excess of 80 "Mph".  I am unsure how this would translate to a real vehicle, but in the simulator it works pretty well with less steering stutter compared to a PID controller.

Here is an example of simplified state: 
![alt text][state]

Function variable definitions or state:
* position coordinates = x, y
* orientation angle = ψ
* velocity = v
* cross-track error = cte 
* psi error = eψ
* steering angle = δ (limited to 25 degrees positive or negative)
* acceleration = a
* distance front to center of gravity which is important in determining turn rate = Lf (2.67)

![alt text][model]

## Additional Changes

    // Cost
    for (int t = 0; t < N; t++)
    {
      fg[0] += 2800 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += 2800 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize actuations
    for (int t = 0; t < N - 1; t++)
    {
      fg[0] += 4.8 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 4.8 * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++)
    {
      fg[0] += 275 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 9.5 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

## Timestep Length and Elapsed Duration (N & dt)

Values used for the timestep length and elapsed duration.
 * timestep length = N = 10 
 * elapsed duration = dt = 0.1 
 * product of N and dt = T = 10 * 0.1 = 1
 N is important because we can set a limit on how far we want to plan.  dt helps us in limiting steering acctuations by timestep.  During the tuning process I would with the assumption keeping T below 3 seconds is optimal.  Also another assumption I operated under is that N should be larger then dt because of the requirement where T shouldn't be greater then 3.  Several values worked between 10, 15, 20 for N, but while mapping lines to the track it was obvious this was too far into the horizon and things looked sporadic. In order to keep N lower then 3 I started with decimals at .3 and then worked lower.  I settled at 0.1 because it was a nice round number and seemed to work well and meet all previous conditions. 

 ## Polynomial Fitting and MPC Preprocessing
 The goal of fitting this polynomial is to minimize the cost function.  Polynomial fitting is useful because they fit most roads.  The waypoints were given in global coordinates so they were converted to vehicle's coordinate system.  So I was able to use the lake_track_waypoints.csv to create a reference point and fit it to the road with the Eigen::VectorXd polyfit function.  Once I did this I could use MPC to minimize the cross track error and the oreientation error over time.  

 Cross track error is defined as :
 * cte_t+1 = f(x_t) − y_t + (v_t * sin(eψ_t) * dt)
   * f(x_t) - y_t = current cross track error
   * (v_t * sin(e_ψ_t) * dt) = change in error caused by vehicle movement

 Orientation Error
 * eψ_t + 1 = ψ_t − ψdes_t + (v_t/Lf * δ_t * dt)
   * ψ_t − ψdes_t = current orientation error 
     * ψdes_t is calculated using the tangential angle polynomial
       * CppAD::atan(3 * coeffs[3] * pow(x0, 2) + 2 * coeffs[2] * x0 + coeffs[1]);
   * v_t/Lf * δ_t * dt = change in error caused by the vehicle's movement.

## Model Predictive Control with Latency

To figure out what would happen with some latency I multipled 0.1 to the x1, y1, psi1, and v1.  After that I calculate the cte and send the state to the MPC solver.


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
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
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