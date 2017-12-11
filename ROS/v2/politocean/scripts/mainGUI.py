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

#set the node name
nodeName = 'GUI'
rospy.init_node(nodeName, anonymous=False)

#function to check if the ROV is awake
def checkRovAwakeness(window, rov, first):
    #check ROV
    while not rospy.is_shutdown():
        sleep(0.5)
        tic = timeit.default_timer()
        while not rov.isAwake(): #if it's not awake
            sleep(0.1)
            if timeit.default_timer()-tic >= 3+3*(1-first): #after a bit
                tic = timeit.default_timer()
                publishErrors(nodeName, "ROV is not responding") #print on console
            sleep(0.1)
            if first:
                rov.sendCommand("W") #send "awake" command
        if first:
            break

#function to init ROV and send commands
def initAndSend(window, rov):
    #active "commands" topic
    for i in range(1,10):
        rov.sendCommand("W")
        sleep(0.2)

    checkRovAwakeness(window, rov, True) #wait for ROV

    #start a new thread to check ROV
    try:
        thread.start_new_thread(checkRovAwakeness, (window, rov, False, ))
    except:
        print("Unable to start checkRovAwakeness thread. Exit")
        sys.exit(-1)

    #send commands
    while 1:
        comm = window.getCommands() #get commands from console
        if comm!="":
            rov.sendCommand(comm)   #send commands to ROV
        sleep(0.1)

def main():
    app = QtGui.QApplication(sys.argv) #gui app init

    #window object
    window = Window(None)
    window.show() #Initialize Window, with disabled objects

    #Initialize the ROV
    rov = ROV(window)

    #init a thread passing window and rov as arguments
    try:
        thread.start_new_thread(initAndSend, (window, rov, ))
    except:
        print("Unable to start the commands thread. Exit")
        sys.exit(-1)

    #start the gui app
    n=app.exec_()

    #when it will be closed, kill other processes and exit
    os.kill(int(sys.argv[1]), signal.SIGKILL)
    os.kill(int(sys.argv[2]), signal.SIGKILL)
    sys.exit(n)

if __name__ == '__main__':
    main()
