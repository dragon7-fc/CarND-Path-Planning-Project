# **Path Planning**

[//]: # (Image References)

[image1]: ./images/demo.gif "demo"
[image2]: ./images/demo2.gif "demo2"
[image3]: ./images/import.jpg "import"
[image4]: ./images/existing_project.jpg "existing project"
[image5]: ./images/select_project.png "select project"
[image6]: ./images/final.png "final"
[image7]: ./images/build_all.png "build all"
[image8]: ./images/run_as.png "run as"
[image9]: ./images/simulator.png "simulator"
[image10]: ./images/state_machine.png "state machine"
[image11]: ./images/spline.png "spline"
[image12]: ./images/result.png "result"

**Path Planning Project**

## Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

Car traversing its given path points
![alt text][image1]

![alt text][image2]

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
    ![alt text][image3]
    3. Select `General > Existing projects into workspace`
    ![alt text][image4]
    4. **Browse** `/root/workspace/CarND-Path-Planning-Project/build` and select the root build tree directory. Keep "Copy projects into workspace" unchecked.
    ![alt text][image5]
    5. Now you should have a fully functional eclipse project
    ![alt text][image6]

3. Code Style

    1. From Eclipse go to `Window > Preferences > C/C++ > Code Style > Formatter`
    2. Click Import
    3. Select `/root/workspace/eclipse-cpp-google-style.xml`
    4. Click Ok

4. Build

    * Select `Project -> Build All`
    ![alt text][image7]

    __OPTIONAL__: Build on command line
    ```
    cd /root/workspace/CarND-Path-Planning-Project/build
    make
    ```

5. Run

    * `Right click Project -> Run as -> 1 Local C++ Application`
    ![alt text][image8]

    __OPTIONAL__: Run on command line
    
    `./path_planning`


6. Launch simulator

    ![alt text][image9]

---
## Implementation

To implement this project, I modified `h.onMessageO()` function of [main.cpp](src/main.cpp), in line 248 ~ 439. To create a smooth trajectory, I used the [Cubic Spline interpolation in C++](https://kluge.in-chemnitz.de/opensource/spline/), as Udacity's recommendation. My implementation mainly involve below two categories.

#### 1. Behavior Planning
In this stage, I will decide what to do, i.e. change lane, accelerate, decelerate, in the near future.

In line 254 ~ 300 of [main.cpp](src/main.cpp) as below, I first detect the lane of ego vehicle and check whether other cars too close, ahead, left or right. If the car is ahead in same lane and within 30 meter of ego vehicle, then `too_close = true`. If they are in left or right lane and within 30 meter of ego vehicle, then `car_left = true` or `car_right = true`.

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

Then in line 302 ~ 331 of [main.cpp](src/main.cpp) as below, I decided what to do in the future. If a car is ahead and too close, and left or right lane is empty, then I decide to change lane. If left or right lane is not empty, then I would like to slow down with -0.5 MPH. If there is no car ahead and center lane is empty, then I would like to change to center lane. And I would like to try to accelerate +0.5 MPH.

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

![alt text][image10]

#### 2. Trajectory Generation
In this stage, I would like to generate trajectory based on the previous lane plan, ego vehicle's coordinate, speed and previous path data points from the planner. 

In line 333 ~ 394 of [main.cpp](src/main.cpp) as below, I pick 5 points for spline trajectory calculation. First two gets from previous path point, or car position if there is no previous trajectory. The remaining three points gets from a far distance (30, 60, 90 meters ahead). To make the work less complicated to the spline calculation based on those points, the coordinates are transformed (shift and rotation) to local car coordinates

``` c++
    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    vector<double> ptsx;
    vector<double> ptsy;

    // Reference x, y, yaw states
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    // If previous size is almost empty, use the car as starting reference
    if (prev_size < 2) {
        // Use two points that make the path tangent to the car
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } else {
        // Use the previous path's endpoint as starting ref
        // Redefine reference state as previous path end point

        // Last point
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        // 2nd-to-last point
        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        // Use two points that make the path tangent to the path's previous endpoint
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Using Frenet, add 30 m evenly spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++) {
        // Shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
    }
```

Finally, in line 396 ~ 439 of [main.cpp](src/main.cpp) as below, I would like to fill in points in the first 30 meter. To keep a continuous trajectory (in addition to adding the last two points of the previous trajectory to the spline adjustment). The previous trajectory points are copied to the new trajectory. The rest of the points are calculated by evaluating the spline and transforming the output coordinates to not local coordinates.  The car is expected to visit each of these points every 0.02 secs, so the number of the points are calculated by `double N = (target_dist/(0.02 * ref_vel/2.24))`.

``` c++
    // Create a spline called s
    tk::spline s;

    // Set (x,y) points to the spline
    s.set_points(ptsx, ptsy);

    // Define the actual (x,y) points we will use for the planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Start with all the previous path points from last time
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Compute how to break up spline points so we travel at our desired reference velocity
    double target_x = 30.0;  //m
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
    double x_add_on = 0;

    double N = (target_dist/(.02 * ref_vel/2.24));  //1 m/s = 2.24 MPH

    // Fill up the rest of the path planner to always output 50 points
    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Rotate back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
```

![alt text][image11]

---
## Result
Using the above implementation, the ego-vehicle is able to drive at least 4.32 miles without incidents, including; exceeding the speed limit, exceeding the max acceleration and jerk, collisions, and going out of lane.

![alt text][image12]
