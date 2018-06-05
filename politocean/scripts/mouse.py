#!/usr/bin/env python

import rospy
import pyautogui
from politocean.msg import *
from errmess_publisher import *
import time

rospy.init_node("mouse", anonymous=False)

def joystickAxisCallback(data):
    global x
    global y
    
    if data.ID == "x_mouse": #laterale
        y = round(data.status - 0.142857149243, 5)
    if data.ID == "y_mouse": #anvanti
        x = round(data.status - 0.142857149243, 5)
    
def joystickButtCallback(data):
    if (data.ID == "mouse_butt") and (data.status == True):
        pyautogui.click(pyautogui.position())
        time.sleep(0.01)

def main():
    global x
    global y
    Ms = 25
    durat = 1
    x = 0
    y = 0
    width, height = pyautogui.size()
    errMessInit() #init topics
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)

    rate = rospy.Rate(500) # 100 Hz
    
    while not rospy.is_shutdown():
      a, b = pyautogui.position()
      
      if a+(x*Ms)>=width and b+(y*Ms)>=height:
       pyautogui.moveTo(width, height)
      if a+(x*Ms)>=width:
       pyautogui.moveTo(width, b+(y*Ms))
      if b+(y*Ms)>=height:
       pyautogui.moveTo(a+(x*Ms), height-5)
      if b+(y*Ms)<=0:
       pyautogui.moveTo(a+(x*Ms))
      if a+(x*Ms)<=0:
       pyautogui.moveTo(0, b+(y*Ms))
      else:
       pyautogui.moveTo(a+(x*Ms), b+(y*Ms))
          
      rate.sleep()
      
if __name__ == '__main__':
    main()
