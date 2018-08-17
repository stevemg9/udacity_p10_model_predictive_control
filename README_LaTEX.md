# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Result Video

[Model Predictive Control](https://youtu.be/eNdUbGSZFvs)

---
## Project Discussion
*Please Note that this markdown file uses LaTEX rendering for equations*

The goal of this project was to develop a Model Predictive Control algorithm to successfully drive a vehicle around the simulate track.  The first step in tackling this problem was to create the model.  In oder to create the model I largely followed the examples and quizzes given in the MPC lessons.  The state vector consisted of 5 variables: X Position, Y Position, Vehicle Orientation, Velocity, Cross Track Error, and Orientation Error. The actuators for this model were acceleration (throttle), and orientation (steering).

$$
state = \begin{bmatrix}
x \\
y \\
\psi \\
v \\
cte \\
\psi_{err} \\
\end{bmatrix}
$$

The state was updated each time step with the following equations:

$$
x_{t+1} = x_t + v_t cos(\psi_t) dt \\
y_{t+1} = y_t + v_t sin(\psi_t) dt \\
\psi_{t+1} = \psi_t + \frac{v_t}{L_f} \delta_t dt \\
v_{t+1} = v_t + a_t dt \\
cte_{t+1} = f(x_t) - y_t + v_t sin(\psi_e) dt \\
\psi_{err_{t+1}} = \psi_t - \psi_{des_{t}} + \frac{v_t}{L_f} \delta_t dt \\
$$
Where,
$$
f(t) = c_0 + c_1 x + c_2 x^2 + c_3 x^3 \\
\psi_{des} = atan(c_1 + 2 c_2 x_t + 3 c_3 x_t^2)
$$
And C0 - C3 are the coefficients for the 3rd order polynomial line fitted by polyfit.

When it came to choosing time steps (N) and the amount of time between steps (dt) I had to do a bit of experimentation.  My original thought was to make the total prediction horizon 4 seconds. My first attempt was N=20 and dt = 2.  This did not work well as the vehicle would plan too far in advance and initiate turns too early and end up cutting the inside corner of turns.  My next thought was to keep the number of time steps the same but lower the prediction horizon to 2 seconds with N = 20 and dt = 0.1.  Still the vehicle was predicting too far ahead and would initiate turns too early.  I knocked N down to 15 and eventually just kept working my way down.  I was surprised that such a low number yielded better results.  My initial thinking was that predicting a path farther out in time would yield more accurate path planning, but the opposite proved true.  My final values were N=7 and dt = 0.1 so the vehicle was only planning 0.7 second in the future.

The only preprocessing that I did on the waypoints was to convert them into the vehicle's local coordinate frame.  This way the vehicle was always at (0,0).  This simplifies the state vector because x, y, and psi are all equal to zero.

The latency issue seemed to be a pretty easy issue to handle.  Since the net effect of the latency is to delay the controls actually being applied.  We can simply predict the next state based on the actuation from 2 time steps ago.  Since I set dt as 0.1s or 100ms, this change handles the 100ms delay perfectly.  Essentially from the above equations, I changed:
$$
a_t = a_{t-1} \\
\delta_t = \delta_{t-1}
$$

This approach seemed to work very well.  I was able to set a target speed of 150mph (though the vehicle never really makes it past about 100mph).  The most important part of my MPC seemed to be the part of the cost function that compounded speed and steering wheel angle.  The penalty was some coefficient * velocity * delta.  This prevents the vehicle from making erratic movements at high speed and significantly stabilized the vehicle path.



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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
