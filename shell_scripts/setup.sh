#!/bin/bash

echo  "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
echo  "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc 
echo  "alias importWorld='( cd \"$HOME/catkin_ws/src/multi_robot_sim/shell_scripts\" && ./import_world.sh )'" >> ~/.bashrc 
