#!/usr/bin/env python
from  pymouse import PyMouse
import rospy
from politocean.msg import *
from errmess_publisher import *

m = PyMouse()
rospy.init_node("mouse_move", anonymous=False)

def joystickAxisCallback(data):
    global x
    global y
    
    if data.ID == "x_mouse": #laterale
        y = round(data.status - 0.142857149243, 5)
    if data.ID == "y_mouse": #anvanti
        x = round(data.status - 0.142857149243, 5)

def main():
    global x
    global y
    Ms=20
    x = 0
    y = 0
    width, height = m.screen_size()
    errMessInit() #init topics
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)
    rate = rospy.Rate(20) # 50 Hz
    
    while not rospy.is_shutdown():
      a, b = m.position()
      if a+(x*Ms)>=width:
       m.move(width, b+(y*Ms))
      if b+(y*Ms)>=height:
       m.move(a+(x*Ms), height)
      if b+(y*Ms)<=0:
       m.move(a+(x*Ms), 0)
      if a+(x*Ms)<=0:
       m.move(0, b+(y*Ms))
      else:
         m.move(a+(x*Ms), b+(y*Ms))
      rate.sleep()
if __name__ == '__main__':
    main()
