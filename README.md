This repository contains implementation of autonomous driving algorithms used to race around a 1/10th model of a F1 car.

## Gap Following

This is my solution for implementing Gap following with clever ideas to help the reactive planner navigate tight turns, and obstacles. Will add testing videos soon!

The repository for this lab is: [here](https://github.com/f1tenth-cmu/f1tenth_lab4)

Race 1 Video: (Best time around the track: 7.8s)

https://github.com/user-attachments/assets/9ed36436-6dc0-4670-9869-3a3bf235cc7e

## Pure Pursuit + SLAM

Simulation Implementation (Includes raceline optimization):

https://github.com/user-attachments/assets/1ad3f71d-b392-4c74-ac99-230474470a85

Race 2 Video: coming on 29th March

### Pure Pursuit Node

Subscribers:

1. Odometry topic (nav_msgs::msg::Odometry): wheel odometry/particle filter

Publishers:

1. Speed and heading angle to drive (ackermann_msgs::msg::AckermannDriveStampe)
2. Visualize corners on Rviz (visualization_msgs::msg::MarkerArray)
3. Visualize lookahead waypoint (visualization_msgs::msg::Marker)

### Helper Scripts

1. **viz_waypoints.py** : Visualize entire waypoints from csv in Rviz.
2. **waypoint_logger.py** : Log waypoints in (x, y, theta, speed) format in csv file reading from a topic.
3. **smoother.py** : Downsampling and smoothing waypoints using slerp.

## RRT Implementation

Using RRT for informed planning and to avoid obstacles. Also experiemnting with RRT*, Kinodynamic RRt, etc.

Simulation Implementation (RRT with local occupancy grid, speed = 2m/s):

https://github.com/user-attachments/assets/ecb37405-a819-4465-889c-1e105e1896d6



