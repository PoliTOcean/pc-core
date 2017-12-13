#!/usr/bin/env python

import rospy
from politocean.msg import *
import pygame
from pygame import locals
import numpy as np

pygame.init()
pygame.joystick.init() # main joystick device system

try:
    j = pygame.joystick.Joystick(0) # istanza del joystick
    j.init()
    print ('Enabled joystick: ' + j.get_name())

except pygame.error:
    print ('no joystick found.')

def joystick_publisher():
    pub = rospy.Publisher('joystick', joystick_data, queue_size=0)
    rospy.init_node('joystick_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    command = joystick_data()
    while not rospy.is_shutdown():
        command.left=j.get_button(0)
        command.up=j.get_button(1)
        command.down=j.get_button(2)
        command.right=j.get_button(3)
        command.l1=j.get_button(4)
        command.l2=j.get_button(5)
        command.r1=j.get_button(6)
        command.r2=j.get_button(7)
        command.select=j.get_button(8)
        command.start=j.get_button(9)
        command.l3=j.get_button(10)
        command.r3=j.get_button(11)
        command.lx=int(round((j.get_axis(0))*100))
        command.ly=int(round((j.get_axis(1))*100))
        command.ry=int(round((j.get_axis(2))*100))
        command.rx=int(round((j.get_axis(3))*100))
        pub.publish(command)
        rate.sleep()
        pygame.event.clear()

if __name__ == '__main__':
    try:
        joystick_publisher()
    except rospy.ROSInterruptException:
        pass
