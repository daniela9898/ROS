#!/usr/bin/env bash

# This script is used to run the project, but does not work yet!!!!!

roslaunch turtlebot3_slam turtlebot3_slam.launch

rosrun map_server map_saver -f /home/felix/catkin_ws/src/christmas_party_simulation/maps/slam

roslaunch turtlebot3_navigation turtlebot3_navigation.launch





