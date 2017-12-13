#!/usr/bin/env python
'''
this node is responsible to get data from a joystick and send them
over the 'joystick' topic. It runs in parallel with joystick_checker, which
tells him the state of the joystick. If it has been disabled, it sops sending
data until it has been re-attached. '''
import roslib
import rospy
from politocean.msg import *
import pygame
from pygame import locals
from std_msgs.msg import String
from time import sleep
import timeit
from errmess_publisher import *

#set the name of the node
rospy.init_node(NODE.JOYPUB, anonymous=False)

#variable check
check = False

#when it receives something over components topic
def checkComponent(data):
    global check
    if not data.ID==ID.JOYSTICK: #if it's not about the joystick
        return  #exit
    if data.status == STATUS.DISABLED: #if it's disabled
        check = False #set check to False
    elif data.status == STATUS.REQUEST: #if it's a request
        if check: #check if it's enabled
            publishComponent(NODE.JOYPUB, ID.JOYSTICK, STATUS.ENABLED)

#subscriber
comp_sub = rospy.Subscriber('components', component_data, checkComponent)

#init the joystick
def joystick_init():
    global check
    j = None;
    #init pygame
    pygame.init()
    pygame.joystick.init() # main joystick device system init
    while not rospy.is_shutdown() and pygame.joystick.get_count()<=0: #loop until it finds a joystick
        pygame.joystick.quit() # main joystick device system quit
        sleep(0.5)
        pygame.joystick.init() # main joystick device system init

    #joystick init
    try:
        j = pygame.joystick.Joystick(0)
        j.init()
        check = True #flag to say that it's enabled
        #publish enabled status
        publishComponent(NODE.JOYPUB, ID.JOYSTICK, STATUS.ENABLED)
        #send a message over 'messages' topic
        publishMessages(NODE.JOYPUB, "JOYSTICK: \""+str(j.get_name())+"\" enabled.")
    except:
        pass
    return j

#publish joystick data
def joystick_publisher(j):
    #set joystick publisher
    pub = rospy.Publisher('joystick', joystick_data, queue_size=0)
    #set ros rate
    rate = rospy.Rate(10) # 10hz
    #prepare variable that has to be sent
    command = joystick_data()
    global check
    #loop until ros is active
    cont = 0
    while not rospy.is_shutdown():
        cont+=1 #counter for the joystick subscriber (see joystick subscriber for info)
        if cont >= 100:
            cont=0
        #save all data
        command.check=cont
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
        try:
            pub.publish(command) #send data over the topic
        except rospy.ROSInterruptException as e:
            publishErrors(NODE.JOYPUB, "Joystick publisher error: "+str(e))
        rate.sleep()
        pygame.event.clear() #clear events
        if not check: #if check hasn't change
            j.quit() #quit
            #send a message
            publishMessages(NODE.JOYPUB, "Joystick has been disconnected.")
            while not rospy.is_shutdown(): #and loop until it has been reconnected
                try:
                    j.init()
                    break
                except:
                    sleep(1)
            #publish messages and status
            publishMessages(NODE.JOYPUB, "Joystick reconnected.")
            check = True #set flag to True
            publishComponent(NODE.JOYPUB, ID.JOYSTICK, STATUS.ENABLED)

if __name__ == '__main__':
    errMessInit() #init topics

    j = joystick_init()
    joystick_publisher(j)
