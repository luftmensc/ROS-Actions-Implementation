# ROS Actions Example Project

This repository demonstrates the implementation of a ROS Action Server and Client using Python, specifically for controlling the Unitree A1 quadruped robot.

## Overview

This project is designed to work with the Unitree A1 quadruped robot, enabling the robot to perform a predefined action of drawing a square using ROS actions. The action server and client communicate to execute movement commands that control the robot's motion.

## Prerequisites

Before using this package, ensure that you have the following installed and set up:

- [ROS (Robot Operating System)](http://wiki.ros.org/ROS/Installation) (compatible version with the A1 robot)
- [Legged Control Framework](https://github.com/qiayuanl/legged_control)

The Legged Control Framework is essential for controlling the Unitree A1 quadruped. Follow the instructions in the linked repository to install and configure the framework.

## Installation

1. Clone this repository into your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository_url>
   ```

2. Build the package using `catkin`:
   ```bash
   cd ~/catkin_ws
   catkin build
   source devel/setup.bash
   ```

## Usage

Once the Legged Control Framework is set up, and your A1 robot is in trot mode, you can start the action server and client as follows:

1. Launch the Action Server:
   ```bash
   rosrun square draw_square_server.py
   ```

2. Launch the Action Client:
   ```bash
   rosrun square draw_square_client.py
   ```

The client will send a goal to the server to command the A1 robot to draw a square with the specified side length and speed.

## Notes

- Ensure the A1 robot is properly configured and in a safe environment to perform the square-drawing action.
- Adjust the side length and speed in the client code if necessary to suit your operational requirements.

