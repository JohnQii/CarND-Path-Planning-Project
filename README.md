# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program, the project from udacity! It's so great.
We get the original project from [udacity](https://github.com/udacity/CarND-Path-Planning-Project)
And I finish it with below steps:
### 1. Make the change line decider.
Just as a behavior planner(FSM), I only use few states to record the car's current. And make the decision that the car should change to left line, right line or non. It's decided by the speed of obstacle which block our cars, the safe distance of left/right line. And the left line has higher priority.
### 2. Limit the speed by safe distance and speed differences.
Get the speed limit by the obstacle which is front of the ego, and the speed is lower than ego's speed. And we will slow down when change line.
### 3. Get anchor points and transfrom it to local coordinate.
Get the anchors points in every 30m, and transform the anchor point to local coordinate. 
### 4. Get cubic spline.
Get the cubic spline by `spline` [lib](https://kluge.in-chemnitz.de/opensource/spline/)
### 5. Get final path points.
Use the speed to calculate distance between two points, and publish the path by Json.
Here is some results:
[successful_image1]
[successful_image2]
### Simulator.
I download the simulator from [udacity]((https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2))

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```
Or you can open a terminal and input as below
```shell
sudo chmod 777 {simulator_file_name}
```
And I suggest that the better way to run the simulator is in the terminal.
### Goals
In this project, our goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
1. The car should try to go as close as possible to the 50 MPH speed limit.
2. The car can overtake the car which is slower than itself.
3. The car should avoid hitting other cars.
4. The car should be able to make one complete loop around the 6946m highway.
5. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop.
6. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Shortages
In this project, I achieve the above goals. But I think we had many shortages which can be completed in the future.
1. I didn't use the prediction, only use the speed of other cars.
2. Speed planning is very important in the motion planning, and I didn't achieve it.
3. The envirment don't has static obstacles, such as conical buckets.
4. The obstacle in the program is only a point, and not a rectangle.
5. THe decider is very simple, only can deal with simple scene.


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

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.
## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


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
* eigen3 
    *linux: I only test on the ubuntu 16.04
	```
	sudo apt-get install libeigen3-dev 
	```

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

[//]: # (Image References)
[successful_image1]: ./data/pictures1.png
[successful_image2]: ./data/pictures2.png
