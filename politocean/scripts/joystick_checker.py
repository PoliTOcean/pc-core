#!/usr/bin/env python
'''
This script runs in parallel with the others (especially it is needed
for the 'joystick_publisher' node) and checks if the joystick is attached
to the system. Asap it isn't anymore, it will update component status over the
components topic, which will stop sending data over its topic until
the joystick is attached again
'''
import roslib
import rospy
import pygame
from pygame import locals
from std_msgs.msg import String
from time import sleep
from errmess_publisher import *
from politocean.msg import *

#the name of the node
rospy.init_node(NODE.JOYCHK, anonymous=False)

def main():
    errMessInit() #init topics

    pygame.init()
    pygame.joystick.init() # main joystick device system init
    cont = 20   #set to 20 to publish disabled status asap
    while not rospy.is_shutdown():
        sleep(0.1)
        pygame.joystick.quit() # main joystick device system quit
        sleep(0.1)
        pygame.joystick.init() # main joystick device system init
        if pygame.joystick.get_count()<=0:
            if cont>=20: #publish status every 2 seconds (20*0.1)
                publishComponent(NODE.JOYCHK, ID.JOYSTICK, STATUS.DISABLED)
                cont=0 #reset to 0 to (re)start the counting
            else:
                cont+=1 #increment
        else:
            cont=20 #reset to 20 to publish disabled status asap


if __name__ == '__main__':
    main()
