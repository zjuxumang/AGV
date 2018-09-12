#!/bin/bash

rosrun map_server map_saver -f ~/catkin_ws/src/my_nav/maps/mymap
rosnode kill slam_gmapping
play ~/catkin_ws/map_saved.wav
exit 0