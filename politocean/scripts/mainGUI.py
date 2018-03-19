#!/usr/bin/env python
'''
This node has the window and ROV objects. Window is the GUI responsible,
while ROV is the responsible of ROS communications.
It has three threads:
 - ROV awakeness checker
 - string commands sender
 - gui
'''
from PyQt4 import QtGui
import sys
from ROV import ROV
from gui import Window
from time import sleep
from std_msgs.msg import String
import timeit
import thread
import rospy
import os
import signal
from errmess_publisher import *
from politocean.msg import *

#set the node name
rospy.init_node(NODE.GUI, anonymous=False)

def exit(n):
    #when it will be closed, kill other nodes and exit
    os.system("rosnode kill "+NODE.JOYPUB)
    os.system("rosnode kill "+NODE.GUI)
    sys.exit(n)


#function to init ROV and send commands
def initAndSend(window, rov):
    os.system("rosrun politocean joystick_publisher.py &")

    #active "commands" topic
    for i in range(1,4):
        rov.sendCommand("W")
        sleep(0.3)

    #while to check if the ROV is awake, as well as tell the ROV that GUI is awake
    while not rospy.is_shutdown():
        sleep(0.5)
        tic = timeit.default_timer()
        rov.sendCommand("A");   #tell the ROV that GUI is awake
        while not rospy.is_shutdown() and not rov.isAwake(): #if it's not awake
            sleep(0.1)
            if timeit.default_timer()-tic >= 5: #after a bit
                tic = timeit.default_timer()
                publishErrors(NODE.GUI, "ROV is not responding") #print on console
                publishComponent(NODE.GUI, ID.ATMEGA, STATUS.DISABLED) #publish ATMega status
            sleep(0.1)

def main():
    app = QtGui.QApplication(sys.argv) #gui app init

    #window object
    window = Window(None)
    window.show() #Initialize Window, with disabled objects

    #init rov object
    rov = ROV(window)

    #init a thread passing window object as argument
    try:
        thread.start_new_thread(initAndSend, (window, rov, ))
    except:
        print("Unable to start the commands thread. Exit")
        exit(-1)

    #start the gui app
    n=app.exec_()

    #when it's closed, send stop signal to the ROV
    rov.sendCommand("SSS")

    exit(n)

if __name__ == '__main__':
    main()
