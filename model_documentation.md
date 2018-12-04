## Writeup

---

**Path Plannning Project**

The goals / steps of this project are the following:
* Implement Path Planing algorithm in C++ which can drive 1 lap around without no incident (exceeding speed/acceleration/jerk, collision, driving out side the lane).
* In this program, car needs to change the lane if it makes sense.
* Write how I generate path.

#### [Rubric](https://review.udacity.com/#!/rubrics/1971/view)

[//]: # (Image References)

[image1]: ./fig/map_waypoint.png "Way point in this project"
[image2]: ./fig/map_waypoint_detail.jpg "Way point in closed view "




### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Discussion
I implemented the path planer using the udacity provided template. I added 2 additional program file named `helper.cpp` and `cost.cpp`. In helper.cpp, there are some helper function's like unit transform, coordinate transform, Jerk Minimizing Trajectory, polynominal computation(computing value and differentiation), getting surrounding car coordinate  and speed in Frenet, and generationg candidates of trajectory. In `cost.cpp`, there are the function relating cost computation.

#### 1. Here I briefly discuss how I generate optimal path.
My program is composed of 5 parts, "Initialization and get last path information", "Sensor fusion", "Trajectory generation", "Optimal path selection with cost function", "Frenet -> Cartesian Coordinate transform".
 1. My progran initialize parameter and get previous path information and append 300 ms path (15 point) from it(`./src/main.cpp` 184-204) for reusability. Because of this, my program has 500 ms latency, however it is not problematic.
 2. My program gets information of the surrounding car. My `surrounndingCarState()` function obtain the information the neareset car front and behind in each lane. I use infromation in 50 m for collision check and 100 m for lane speed check. Using front cars' speed in 100 m, I get the preferable lane whose speed is highest(`./src/main.cpp` 124-233). In efficiency stand point, it is better to run in the lane.
 3. I get candidates of trajectory(`./src/main.cpp` 246)using `generateTrajectories()`. In my program, trajectory is computed from target speed and target lane after 2 s. Start position, velocity and acceleration are computed from previous path. I prepare 2 end configuration. When speed is more than 10 mph, target position in s is (initial value) + T(target vel) which means car keep same speed if target speed and current speed are same, otherwise it is (initial value) + 2.0T(target vel) and target position d is the same. This is because in low speed, large d change is not possible because of the nonholonomics of the car.
 Then, quintic polynominal trajectory is computed by means of Jerk-Minimizing-Trajectory method. In each lane, 5 paths are computed.
 - Keep Speed (target velocity is constant)
 - high acceleration and low acceleration(target velocity + 0.4574, 0.2237)
 - high deceleration and low deceleration(target velocity - 0.4574, 0.2237)
 4. Using cost function, I get the optimal path from candidates of trajectory. Detail of this is mentioned below.
 5. From computed optimal path in Frenet coordinate, I get the optimal path in Cartesian coordinate. I used `getXYspline()` function(`./src/main.cpp` 43-51) for this to prevent uncontinuity when the car pass way points. I used interpolated map waypoint in `./data/precise_highway_map.csv`. This is calculated with getFinerWaypoint.py. Scipy provides interpolation function named [`splprep()`](https://docs.scipy.org/doc/scipy-0.18.1/reference/generated/scipy.interpolate.splprep.html#scipy.interpolate.splprep). I chose 3 DOF spline to generate it. (Actually, this causes the outside the lane problem in high curvature because of the coordinate transformation difference between unity code and my program. This is compensated in `lane_d()` (`helper.cpp`)) The result is below

![alt text][image1]

![alt text][image2]


In summary, I used Jerk-Minimizing-Trajectory and cost function to plan optimal path.


#### 2. Here I briefly discuss the detail of my cost function
I use 4 cost function in `cost.cpp` (In fact, I made 8 but 4 of them are not used). All cost function are normalized between 0 and 1.
	-`costMaxAcc()` penalizes the highest accleration in the trajectory. This checks the feasibility, therefore this is a binary cost function. 
	-`costSafety()` penalizes thedistance between nerest car in the same lane. This checks the safety and this is continuous function. Distance more than 30 m(about possible 1 s position difference in 50 mph) is the 0, and that of 0 m is 1. This is quadratic function so that the cost becomes quickly larger if the other car comes nearer.
	-`costSpeed()` penalize the speed lower than 46 mph, and higher than 50 mph. This checks leagality and efficiency. 46 mph derives from the test. I compute speed from the Frenet s coordinate so there are some errors from coordinate transformation especially the way with high curvature. This is also quadratic function. The value is 0 when the speed is 46 mph, and 1 when the speed is 0 and higher than 50 mph.
	-`costLaneEfficiency()` penarizes the Frenet d distance between current position and preferable lane. Preferable lane is the highest speed lane. This checks the efficiency of the trajectory and owing to this function, the car can change lane. This is quadratic function and it is 0 if the preferable lane and current position are same, and 1 if the d distance is 12 (3 lane width).

#### 3.  Here I briefly discuss how I tune each parameter.
This is the final parameters I have tuned.

| Parameters    | Values    |
|:-------------:|:-------------:|
| The number of points last trajectory are used  [-]    |  15 (300ms)     |
| The number of trajectory points   [-]                 |  50(1000ms)     |
| Trajectory duration [s]                               |  2              |
| Acceleration of trajectory candidate [m/s^2]          |  0.4457, 0.2237 |
| Acceptable speed [mph]                                |  46             |
| Acceptable acceleration [m/s^2]                       |  8              |
| The range in which collisions are checked             |  30.0           |
| The range in which the lane speed is checked          |  100.0          |

| Weights    | Values    |
|:-------------:|:-------------:|
| Max acceleration      |  1000     |
| Safety                |  ?        |
| Speed                 |  ?        |
| Lane efficiency       |  1        |

Here is a link to [my video result](./videos/Path-Planning.mp4).

I manually tune my parameters.
I normally use the parameter that is just more than range in 1 s motion. And max speed and acceleration are chosen so that it can tolerate the error from coordinate transform.
Weights are derived from test and priorities. Priorities are set in the following order. Feasibility >> Safety >> Leagality >> Efficiency. Then I tested and finally chose the parameter above.

#### 3. Here I briefly discuss potential improvement of my program.

There are some ways to improve my program. First, I used constant velocity model in sensor fusion, however this cause some problem. For example, if the front car stops suddenly, the path planner cannot deal with it immediately because it reacts due to the speed. If I use the model considering accelaration, more safety can be realized. Second, my car sometimes changes double lane. In real situation it is possibly dangerous. My car should change one lane by one lane. If I implement Finite-State-Machine, this could be possible. Third, my car sometimes oscilates. For example, when the 2 car runs at almost same speed in front of my car, then the fastest lane changes continuously. It causes my car's preference changes and moves oscillatorily. I can possibly deal with this by using cost function that penalizes the number of lane changes in unit time.
