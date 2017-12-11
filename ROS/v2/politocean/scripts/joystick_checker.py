#!/usr/bin/env python
'''
This script runs in parallel with the others (especially it is needed
for the 'joystick_publisher' node) and checks if the joystick is attached
to the system. If it is, sends a message over a topic that is received
from joystick_publisher, which will stop sending data over its topic until
the joystick is attached again
'''
import roslib
import rospy
import pygame
from pygame import locals
from std_msgs.msg import String
from time import sleep
from errmess_publisher import *

#the name of the node
rospy.init_node('joystick_checker', anonymous=False)

#ROS publisher
pub = rospy.Publisher('joystick_checker', String, queue_size=0)

def main():
    sleep(2) #wait a bit to let the other nodes to start
    pygame.init()
    pygame.joystick.init() # main joystick device system init
    while not rospy.is_shutdown():
        sleep(0.1)
        pygame.joystick.quit() # main joystick device system quit
        sleep(0.1)
        pygame.joystick.init() # main joystick device system init
        if pygame.joystick.get_count()>0: #if there is a joystick
            pub.publish("Y")    #send message over the topic


if __name__ == '__main__':
    main()
