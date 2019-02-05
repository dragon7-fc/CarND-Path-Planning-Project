# **Path Planning**

[//]: # (Image References)

[image1]: ./images/demo.gif "demo"
[image2]: ./images/import.jpg "import"
[image3]: ./images/existing_project.jpg "existing project"
[image4]: ./images/select_project.png "select project"
[image5]: ./images/final.png "final"
[image6]: ./images/build_all.png "build all"
[image7]: ./images/run_as.png "run as"
[image8]: ./images/simulator.png "simulator"
[image9]: ./images/state_machine.png "state machine"
[image17]: ./images/result.png "result"

**Path Planning Project**

## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

Car traversing its given path points
![alt text][image1]

### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Simulator behavior

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

---
## Dependencies

* [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

## Environment Setup

1. Open Eclipse IDE

__Linux__:
```
docker run --rm --name planning \
    --net=host -e DISPLAY=$DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority \
    dragon7/carnd-path-planning-project
```

__Mac__:
```
socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"

docker run --rm --name planning \
    -e DISPLAY=[IP_ADDRESS]:0 \
    -p 4567:4567 \
    dragon7/carnd-path-planning-project
```

2. Import the project into Eclipse

    1. Open Eclipse.
    2. Import project using Menu `File > Import`
    ![alt text][image2]
    3. Select `General > Existing projects into workspace`
    ![alt text][image3]
    4. **Browse** `/root/workspace/CarND-Path-Planning-Project/build` and select the root build tree directory. Keep "Copy projects into workspace" unchecked.
    ![alt text][image4]
    5. Now you should have a fully functional eclipse project
    ![alt text][image5]

3. Code Style

    1. From Eclipse go to `Window > Preferences > C/C++ > Code Style > Formatter`
    2. Click Import
    3. Select `/root/workspace/eclipse-cpp-google-style.xml`
    4. Click Ok

4. Build

    * Select `Project -> Build All`
    ![alt text][image6]

    __OPTIONAL__: Build on command line
    ```
    cd /root/workspace/CarND-Path-Planning-Project/build
    make
    ```

5. Run

    * `Right click Project -> Run as -> 1 Local C++ Application`
    ![alt text][image7]

    __OPTIONAL__: Run on command line
    
    `./path_planning`


6. Launch simulator

    ![alt text][image8]

---
## Implementation

To implement this project, I modified `h.onMessageO()` function of main.cpp, between line 248 ~ 439. To create a smooth trajectory, I used the [Cubic Spline interpolation in C++](https://kluge.in-chemnitz.de/opensource/spline/), as Udacity's recommendation. My implementation mainly involve below two categroies.

1. Behavior Planning

In line 254 ~ 300 as below, I first detect lane of ego vehicle and check whether other cars too close, ahead, left or right. If the car is ahead in same lane and within 30 meter of ego vehicle, then `too_close = true`. If they are in left or right lane and within 30 meter of ego vehicle, then `car_left = true` or `car_right = true`.

``` c++
    // Lane identifiers for other cars
    bool too_close = false;
    bool car_left = false;
    bool car_right = false;

    // Find ref_v to use, see if car is in lane
    for (int i = 0; i < sensor_fusion.size(); i++) {
        // Car is in my lane
        float d = sensor_fusion[i][6];

        // Identify the lane of the car in question
        int car_lane;
        if (d >= 0 && d < 4) {
            car_lane = 0;
        } else if (d >= 4 && d < 8) {
            car_lane = 1;
        } else if (d >= 8 && d <= 12) {
            car_lane = 2;
        } else {
            continue;
        }

        // Check width of lane, in case cars are merging into our lane
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];

        // If using previous points can project an s value outwards in time
        // (What position we will be in in the future)
        // check s values greater than ours and s gap
        check_car_s += ((double)prev_size*0.02*check_speed);

        int gap = 30; // m

        // Identify whether the car is ahead, to the left, or to the right
        if (car_lane == lane) {
            // Another car is ahead
            too_close |= (check_car_s > car_s) && ((check_car_s - car_s) < gap);
        } else if (car_lane - lane == 1) {
            // Another car is to the right
            car_right |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
        } else if (lane - car_lane == 1) {
            // Another car is to the left
            car_left |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
        }
    }
```

Then in line 302 ~ 331 as below, I decided what to do in the future. If a car is ahead and too close, and left or right lane is empty, then I decide to change lane. If left or right lane is not empty, then I would like to slow down. If there is no car ahead and center lane is empty, then I would like to change to center lane. And I would like to try to accelerate.

``` c++
    // Modulate the speed to avoid collisions. Change lanes if it is safe to do so (nobody to the side)
    double acc = 0.224;  //0.224 m/s = 0.5 MPH
    double max_speed = 49.5;
    if (too_close) {
        // A car is ahead
        // Decide to shift lanes or slow down
        if (!car_right && lane < 2) {
            // No car to the right AND there is a right lane -> shift right
            lane++;
        } else if (!car_left && lane > 0) {
            // No car to the left AND there is a left lane -> shift left
            lane--;
        } else {
            // Nowhere to shift -> slow down
            ref_vel -= acc;
        }
    } else {
        if (lane != 1) {
            // Not in the center lane. Check if it is safe to move back
            if ((lane == 2 && !car_left) || (lane == 0 && !car_right)) {
                // Move back to the center lane
                lane = 1;
            }
        }
        
        if (ref_vel < max_speed) {
            // No car ahead AND we are below the speed limit -> speed limit
            ref_vel += acc;
        }
    }
```

![alt text][image9]

2. Trajectory Generation

---
## Result

![alt text][image17]
