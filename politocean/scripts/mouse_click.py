#!/usr/bin/env python
from  pymouse import PyMouse
import rospy
from politocean.msg import *
from errmess_publisher import *

m = PyMouse()
rospy.init_node("mouse_click", anonymous=False)

def joystickButtCallback(data):
    if (data.ID == "mouse_butt") and (data.status == True):
        a, b = m.position()
        m.click(a , b , 1)


def main():
    
    errMessInit() #init topics
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    
    rospy.spin()
    
if __name__ == '__main__':
    main()
