#!/usr/bin/env python
'''
this node is responsible to get data from a joystick and send them
over the 'joystick' topic. It runs in parallel with joystick_checker, which
tells him the state of the joystick. To do that, it increments a variable
check (which is sent over the topic), in order to know if the joystick isn't
attached. In fact, the variable keeps the same value for some seconds, and
that let the node joystick_subscriber to know that the joystick isn't sending
anything. '''
import roslib
import rospy
from politocean.msg import *
import pygame
from pygame import locals
from std_msgs.msg import String
from time import sleep
import timeit

#set the name of the node
nodeName = 'joystick_publisher'
rospy.init_node(nodeName, anonymous=False)

#publishers
errors_pub = rospy.Publisher('errors', String, queue_size=10)
messages_pub = rospy.Publisher('messages', String, queue_size=6)

#variable check
check = 0

#when it receives something from joystick_cheker, increment check
def increment_check(data):
    global check
    check+=1
    if check>=127:
        check=0

#subscriber
joystick_sub = rospy.Subscriber('joystick_checker', String, increment_check)

#function to publish errors over a topic
def publishErrors(err):
    try:
        errors_pub.publish(nodeName+'::'+err)
    except:
        print("!!! Closed errors topic !!!")

#init the joystick
def joystick_init():
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
        #send a message over 'messages' topic
        messages_pub.publish("JOYSTICK: \""+str(j.get_name())+"\" enabled.")
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
    eps = 0.001
    tic = timeit.default_timer() #init the timer
    old_check = -1
    #loop until ros is active
    while not rospy.is_shutdown():
        if check==old_check: #if check hasn't change
            if timeit.default_timer()-tic>=1.5: #for at least 1.5 seconds
                j.quit() #quit
                #send a message
                messages_pub.publish("Joystick has been disconnected.")
                while True: #and loop until it has been reconnected
                    try:
                        j.init()
                        break
                    except:
                        sleep(0.1)
        else: #else, save timestamp
            tic = timeit.default_timer()
        old_check = check #save old_check
        #save all data
        command.check=check
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
            publishErrors("Joystick publisher error. "+str(e))
        rate.sleep()
        pygame.event.clear() #clear events

if __name__ == '__main__':
    sleep(2) #wait for other nodes
    #send ??? to "active" the topics
    for i in range(1,10):
        publishErrors("???")
        messages_pub.publish("???")
        sleep(0.2)
    j = joystick_init()
    joystick_publisher(j)
