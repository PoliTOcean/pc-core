#!/bin/bash

rosrun politocean joystick_publisher.py &
pid1=$!

rosrun politocean joystick_checker.py &
pid2=$!

rosrun politocean mainGUI.py $pid1 $pid2
