#!/bin/bash

path=$( echo $ROS_PACKAGE_PATH | cut -d ':' -f 1 )
roslaunch $path/../launches/video_stream.launch &
pid1=$!
rosrun politocean joystick_subscriber.py &
pid2=$!
rosrun politocean mainROV.py $pid1 $pid2
