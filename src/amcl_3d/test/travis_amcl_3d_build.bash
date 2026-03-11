#!/bin/bash

#catkin setup
mkdir -p ~/catkin_ws/src/amcl_3d

cp -r ./ ~/catkin_ws/src/amcl_3d

cd ~/catkin_ws/src

source /opt/ros/kinetic/setup.bash

catkin_init_workspace

cd ~/catkin_ws

rosdep install --from-paths ~/catkin_ws/src/amcl_3d -y

catkin_make