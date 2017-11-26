# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Reflections
* First I tried basic version of PID controller by manually setting Kp, Ki, and Kd params and calculating the multipliers for the params (namely cte, int_cte, and diff_cte). In this what I have observed is that since the int_cte is accumulating into a bigger number as the time goes by, tau_i (Ki) needs to be very small. By setting Kp, Ki, and Kd to `0.1, 0.001, 0.1` - the vehicle was about to move on the track at ~10 mph speed (setting a throttle values of 0.1). However when I increased the throttle to 0.2 (with a speed of around 20 mph), the vehicle had swift turns about more than 4 radians. Since the expected range for the -1 to 1 radians, I've restricted the bounds to be -1 to 1. However this doesn't help either to restrict the vehicle to be on the track for long time.
* Due to above issues, I've implemented the twiddle algorithm as given in the Lesson 16.12 (PID control) within `PID::UpdateError()` function of PID.cpp.
  * Here is the snippet that computes the `cte, int_cte and diff_cte`
  ```
  if (nCurrentStep == 0 ){
      prev_cte = cte;
  }
  cteSum += cte;
  p_error = cte;
  d_error = cte - prev_cte;
  i_error = cteSum;
  ```
  * Here is the snippet that calculates the cte error for certain number of cycles (similar to what is given in the run function of the sample in Lesson 16.12).

    ```
    //Calculate the total error if the current step is more than <nRunLimit>
    if ((nCurrentStep % (nRunLimit+nRunRangeLimit))  >= nRunLimit){
        dTotalError += std::pow(cte, 2);
    }
    ```
  * Here is the snippet of the code that implements the twiddle code (similar to the python code in Lesson 16.12 example code)

    This snippet verifies two conditions.

        1, Stop the twiddle if the sum of the dp error components is less than 0.1.

        2, Enter the loop after skipping certain iterations (in this case enter the loop after 2 iterations. Tried to increase the number of iterations to about 10 - 100, but ran into issues with the steering angle calculations).

    ```
    //Only enter this loop if the dTotalError is calculated nRunLimit times
    if ((TotalError() > errorTolerance)&&((nCurrentStep % (nRunLimit+nRunRangeLimit))  == nRunLimit)){
    ```

    The following snippet does the params and dp error corrections (The algorithm for this is based on the twiddle code from the Lesson 16.12).

    ```
    switch (tPhase){
        case PHASE1:
            UpdateParamWithdp(pramInProgress, dpList[pramInProgress]);
            tPhase = PHASE2;
            break;
        case PHASE2:
            if (dTotalError < dBestError){
                dBestError = dTotalError;
                dpList[pramInProgress] *= 1.1;
                tPhase = PHASE4;
            } else {
                UpdateParamWithdp(pramInProgress, -2*dpList[pramInProgress]);
                tPhase = PHASE3;
            }
            break;
        case PHASE3:
            if (dTotalError < dBestError){
                dBestError = dTotalError;
                dpList[pramInProgress] *= 1.1;
                tPhase = PHASE4;
            }else {
                UpdateParamWithdp(pramInProgress, dpList[pramInProgress]);
                dpList[pramInProgress] *= 0.9;
                tPhase = PHASE4;
            }
            break;
        case PHASE4:
            tPhase = PHASE1;
            pramInProgress = (pramInProgress+1)%3;

            cout << "Current Step at which the param updates are done" << nCurrentStep << endl;
            cout << "After param update dp coeff" << dpList[0] << ", " <<  dpList[1] << ", " << dpList[2] << endl;
            cout << "After param update K coeff" << Kp << ", " << Ki << ", " << Kd << endl;
            break;

    }
    ```

* Even with the twiddle algo code, after few twiddle updates, the steering anlge is going wild (turning about 4.4 radians). After looking at the `pid params, and cte, int_cte, diff_cte` values, what I found is that the Ki is about 0.5 and the int_cte is going to be a big number ( as it is a cumulative sum). To reduce the impact of `int_cte`, I've tried to look into past 10 values for the int_cte values. Even in this case, it is giving wild steering values after some time as well. For this reason, I've decided to not include the Ki to predict the steering angle.

  ```
  steer_value = (- (pid.Kp*pid.p_error /* + pid.Ki*pid.i_error */ + pid.Kd*pid.d_error));
  //constrain the values between -1 to 1
  steer_value = std::max(std::min(1.0, steer_value), -1.0);
  ```
Now the vehicle was able to complete the track. However the speed of the vehicle is about 10 mph. When increased the speed to 20 mph (by changing the throttle to 0.2), the vehicle is moving faster but going wild when there are turns on the track. To solve this problem, `I've modified the code such that when the predicted steering value (absolute) is more than 0.18 radians (10 degrees)`, I am reducing the throttle value to 0.1 at that instance and rolling it back to the throttle value of 0.2 when the steering value is less than 0.18 radians.

* One other change I did inside the twiddle algorithm is that reducing number of iterations before twiddle params are updated to 2 (as opposed to 200 in the lesson). When I tried 200 samples to use existing params before using twiddle to update params caused wild steering angles (more than 4 radians). Because the twiddle params are updated more frequently, there is more compute involved. However the params are stabilized around `580 steps` (which is about `20% time into the time it takes to complete one turn of the track`). At this time, the sum of param errors (dp error) is under 0.1, and the params are finalized at `0.719079, 0.766853, 0.902407 (Kp, Ki, Kd)`. Here is the recorded video of the vehicle on the track  

* Here are some issues I found with the existing PID controller and needs improvement
  * `Vehicle speed is under 20 mph`. I may need to include a twiddle algorithm to change the throttle values.
  * Still the vehicle steering angle are much larger than the passenger comfort level. When watching the vehicle moving, it causes nausea to me - which is not what is expected from a car on the road. Change/improve the PID controller algorithm (no idea what it would be at this time).
  * Try to include Ki into the `steer_value` prediction.


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
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
