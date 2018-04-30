#!/usr/bin/env python

import rospy
from politocean.msg import *
from errmess_publisher import *

rospy.init_node("mouse", anonymous=False)

def joystickAxisCallback(data):
    global x
    global y
    
    if data.ID == "x_mouse": #laterale
        x = round(data.status - 0.142857149243, 5)
    if data.ID == "y_mouse": #anvanti
        y = round(data.status - 0.142857149243, 5)

def main():
    global x
    global y
    
    x = 0
    y = 0
    
    errMessInit() #init topics
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)

    rate = rospy.Rate(50) # 50 Hz
    
    while not rospy.is_shutdown():
        print x, y

if __name__ == '__main__':
    main()