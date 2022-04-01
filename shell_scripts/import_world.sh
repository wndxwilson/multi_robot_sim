#!/bin/bash

echo Select file name:

read filename

cd ~/osm2gazebo
cp "$filename.yaml" "$filename.gpickle" ~/catkin_ws/src/multi_robot_sim/maps
cp "$filename.sdf" ~/catkin_ws/src/multi_robot_sim/worlds
echo copied