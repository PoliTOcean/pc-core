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

#set the node name
rospy.init_node('GUI', anonymous=False)

#publishers
messages_pub = rospy.Publisher('messages', String, queue_size=6)
errors_pub = rospy.Publisher('errors', String, queue_size=10)

#publish errors function
def publishErrors(err):
    try:
        errors_pub.publish(err)
    except:
        print("!!! Closed errors topic !!!")

#function to check if the ROV is awake
def checkRovAwakeness(window, rov, first):
    #check ROV
    while not rospy.is_shutdown():
        tic = timeit.default_timer()
        if first:
            rov.sendCommand("W")
        sleep(0.5)
        while not rov.isAwake(): #if it's not awake
            if window.isRunning():
                #set NO SIGNAL image
                window.setCurrentFrame(0, None)
                window.setCurrentFrame(1, None)
                window.setCurrentFrame(2, None)
            sleep(0.1)
            if timeit.default_timer()-tic >= 3+3*(1-first):
                tic = timeit.default_timer()
                messages_pub.publish("ROV is not responding") #print on console
                '''here disable window objects'''
            sleep(0.1)
            if first:
                rov.sendCommand("W")
        '''here enable window objects'''
        if first:
            break
        sleep(0.5)

#function to init ROV and send commands
def initAndSend(window, rov):
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
    #send ??? to "active" topics
    for i in range(1,10):
        errors_pub.publish("???")
        messages_pub.publish("???")
        sleep(0.2)
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
