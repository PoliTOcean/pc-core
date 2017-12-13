#!/usr/bin/env python
'''
This node receives joystick data and converts them into string commands.
If the variable "check" remains the same for some time, it sends a STOP signal
to the ROV, since it means that the joystick has been disconnected'''
import rospy
from std_msgs.msg import String
from politocean.msg import *
from time import sleep
import timeit

#set node name
nodeName = 'joystick_subscriber'
rospy.init_node(nodeName, anonymous=False)

#ROS publishers
errors_pub = rospy.Publisher('errors', String, queue_size=10)
commands_pub = rospy.Publisher('commands', String, queue_size=9)

#check variable
check = 0

#function to publish errors over the ROS topic
def publishErrors(err):
    try:
        errors_pub.publish(nodeName+"::"+err)
    except:
        print("!!! Closed errors topic !!!")

#function that receives joystick data
def joystickCallback(data):
    global check
    check = data.check
    #prepare command string
    comm=""
    if data.start:
        comm+="G" #go
    if data.up:
        comm+="U" #up
    if data.down:
        comm+="D" #down
    if (data.up or data.down) and data.l1>0:
        comm+="FV"+str(int(data.l1)+int(data.l2))+" " #fast vertical
    comm+="RX"+str(data.rx)+"  "
    comm+="RY"+str(data.ry)+"  "
    if data.ry!=0:
        comm+="FH"+str(int(data.r1)+int(data.r2))+" " #fast horizontal
    comm+="LY"+str(data.ly)+"  "
    comm+="LX"+str(data.lx)+"  "
    if data.select:
        comm="SSS" #stop
    try: #publish commands
        commands_pub.publish(comm)
    except rospy.ROSInterruptException as e:
        publishErrors("Commands topic publisher: "+str(e))

def main():
    #send some ??? to "active" topics
    for i in range(1,10):
        commands_pub.publish("SSS")
        publishErrors("???")
        sleep(0.2)

    #subscriber
    joystick_sub = rospy.Subscriber("joystick", joystick_data, joystickCallback)

    global check
    prec_check = -1
    delayed = False
    while True:
        #check for check variable
        if prec_check==check:
            if not delayed:
                commands_pub.publish("SSS")
            delayed = True
        else:
            delayed = False
        prec_check = check
        sleep(1)

if __name__ == '__main__':
    main()
