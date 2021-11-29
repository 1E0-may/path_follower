#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster


#build obstacles file
/usr/bin/python build_obstacles.py

# simulation launch
X_INIT=$(cat src/rosbot_navigation/src/path.csv | cut -d "," -f 3)
Y_INIT=$(cat src/rosbot_navigation/src/path.csv | cut -d "," -f 4)
roslaunch rosbot_description rosbot.launch x_init:=$X_INIT y_init:=$Y_INIT