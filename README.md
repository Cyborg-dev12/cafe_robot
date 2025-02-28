# Cafe Robot ROS Project

## Overview
This ROS project implements a robotic butler for the French Door Café. The robot navigates between a home position, kitchen, and three customer tables (table1, table2, table3) to deliver food orders. It handles single and multiple orders, confirmations, timeouts, and cancellations using a state machine built with SMACH.

## Features
- **Single Order Delivery**: Moves from home to kitchen, then to the specified table, and back to home.
- **Multiple Orders**: Delivers to multiple tables in sequence, skipping canceled orders.
- **Confirmation Handling**: Waits for kitchen and table confirmations with timeouts.
- **Cancellation Support**: Returns to kitchen or home based on cancellation timing.

## Prerequisites
- **OS**: Ubuntu 18.04
- **ROS Version**: Melodic
- **Dependencies**: `rospy`, `smach`, `smach_ros`, `move_base_msgs`, `actionlib`, `std_msgs`
- **Simulation**: TurtleBot3 (Gazebo)

## Installation
1. Clone this repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Cyborg-dev12/cafe_robot.git

2. Dependencies
   ```bash
   sudo apt install ros-melodic-turtlebot3 ros-melodic-turtlebot3-simulations ros-melodic-turtlebot3-navigation ros-melodic-gmapping ros-melodic-dwa-local-planner

3. Build the Workspace
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash

## Usage
1. Launch Gazebo Simulation
   ```bash
   export TURTLEBOT3_MODEL=burger
   roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch gui:=false

2. Run SLAM
   ```bash
   roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

3. Start Navigation
   ```bash
   roslaunch turtlebot3_navigation move_base.launch

4. Run State Machine
   ```bash
   rosrun cafe_robot cafe_robot_smach.py

5. Send Orders
   ```bash
   rostopic pub /orders std_msgs/String "table1 table2"
   rostopic pub /kitchen/confirmation std_msgs/String "ready"
   rostopic pub /table1/confirmation std_msgs/String "delivered"
