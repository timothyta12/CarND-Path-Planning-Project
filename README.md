# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Project Rubic 
This rubic was used as a guideline for the project
[Rubic](https://review.udacity.com/#!/rubrics/1971/view).

## The Project
Many parts of the self-driving system have been implemented already. The focus of this project is to create the car's trajectory.

### Gathering Information
In order to create a trajectory, the car needs to know certains things about the road.

#### Assumptions
For the purposes of simplicity, the car's sensors has gathered perfect information about its whereabouts, along with other car's locations, and has perfect road knowledge.

#### Vehicle Awareness (Lines 264-307) and Changing Lanes (Lines 309-337)
In order to avoid collisions and change lanes, the car must be able to sense the cars around itself.
The car simply checks whether a car in front of it is in a certain range. If a car is within this range, a flag is set for the car to slow down and search for a lane change.

A lane change is permissible if there is space both in front and behind the car in the next lane over.

This information is conveyed to the trajectory calcuation by specifying a goal velocity and desired lane position.

### Calculating Trajectory (Lines 342-438)
With the desired velocity and lane obtained above, a tracjectory can be calculated for the car. Using the [Spline](http://kluge.in-chemnitz.de/opensource/spline/) library, an equation for the trajectory is created by interpolating points.

First, to create a smooth corherent path, the unused previous path's information is perserved. If there is no previous path, the car's current location is used as a starting reference.

In addition to these starting points, the car plots several points 30m, 60m, and 90m ahead of itself in the desired lane. (Lines 377-388)

These points are then normalized such that the starting points are at _x = 0_, _y = 0_ and _yaw = 0_. This simplifies the calculations of the trajectory.

A spline is fitted to these sets of points. The spline guarentees that we will visit each of these points.

Finally, using the spline, we calculate various points by considering the velocity and time elapsed for each car controller update. Keep in mind that the points must be denormalized back to the world's coordinate system. (Lines 415-438)

### Considerations and Problems
With this model, the car is able to navigate the highway well. The paths created are smooth and not jerky. The car will avoid crashing into a car in front of it and is able to execute lane changes to attempt to go faster. This car passed all parts of the project except for rare occasions.

One issue with this model is that cars are not predicted very well. The model only considers other car's current location and current velocity. It does not consider acceleration and thus does not accurately predict the car's future location. The longer the latency, the bigger the discrepency.

This issue has two major implications. First, the model is aggressive in staying away from the car in front of it. It assumes that the other car will slow down and thus, applies the brakes aggressively. The car will not smoothly follow the car at constant speed as it accelerates and decelerates to keep the car in front of it at a target distance. Although it is not a fatal problem, passengers of the car will be very upset.
The next problem caused by this is when two cars try to merge into the same lane. The model does not predict whether the car in front of it will change lanes. This error will cause a collision, which rarely, but could happen. People will often make this mistake causing crashes too. Both driver's check the lane they want to go into, failing to look at the car two lanes over.
This problem can be fixed by having a more intelligent prediction system and a better sense of other car's actions.

Another efficieny problem is that the lane change decision making is very simple. A lane change will occur if the car in front of it is below the speed limit and it is safe to do so. This can be fixed through a cost function that weighs between staying in a lane or switching lanes.