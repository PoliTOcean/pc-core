#!/usr/bin/env python
'''This is the main node that communicates with ROS over the ROV Raspberry.
It receives commands to write over arduino and sends sensors data (or errors).
There is a system to wait for Arduino plug in and manage its disconnection.
'''
import roslib
import rospy
import roslaunch
from sensor_msgs.msg import Image
from std_msgs.msg import String
import serial
import timeit
import signal
import sys
import os
from time import sleep
from politocean.msg import *



#arduino init
#arduino=serial.Serial("/dev/ttyUSB0",9600)
#arduino=serial.Serial("/dev/cu.usbserial-A900XW2W",9600)

#set node name
nodeName = 'ROV'
rospy.init_node(nodeName,anonymous=False)

#publishers
#sensors
sensors_pub = rospy.Publisher('sensors', sensors_data, queue_size=3)
#errors
errors_pub = rospy.Publisher('errors', String, queue_size=9)
#messages
messages_pub = rospy.Publisher('messages', String, queue_size=5)

#global variables
arduino=None
ready=False #tell if Arduino is ready

#publish messages over topic
def publishMessages(data):
    try:
        messages_pub.publish(str(data))
    except rospy.ROSInterruptException as e:
        publishErrors("Messages publishing error: "+str(e))

#take commands and send to Arduino
def commandsCallback(data):
    if "W" in str(data):
        publishMessages("awake")
    elif ready: #if ready, write on the Serial
        try:
            arduino.write("/ "+str(data).encode()+" \\")
        except Exception as e:
            publishErrors("Unable to write on arduino: "+str(e))

#publish errors over the topic
def publishErrors(err):
    try:
        errors_pub.publish(nodeName+"::"+err)
    except:
        print("!!! Closed errors topic !!!")

#publish sensors data over the topic
def publishSensors(sens):
    try:
        sensors = sensors_data()
        values = sens.split(' ')
        try: #catch exceptions for some data errors
            sensors.pressure = float(values[0])
            sensors.temperature = float(values[1])
            sensors.pitch = float(values[2])
            sensors.roll = float(values[3])
        except:
            pass
        sensors_pub.publish(sensors) #publish
    except rospy.ROSInterruptException as e:
        publishErrors("Closed sensors topic: "+str(e))

#subscribers
#commands topic
commands_sub = rospy.Subscriber("commands", String, commandsCallback)

#function called at the exit
def exit_signal(signum, frame):
    #kill other processes
    global arduino
    arduino = None
    os.kill(int(sys.argv[1]), signal.SIGTERM)
    os.kill(int(sys.argv[2]), signal.SIGKILL)
    os.kill(int(os.getpid()), signal.SIGKILL)
    sys.exit(0)

def main():
    #prepare to catch termination signals
    signal.signal(signal.SIGINT, exit_signal)
    signal.signal(signal.SIGTERM, exit_signal)
    global arduino
    #send ??? to "active" topics
    for i in range(1,10):
        publishErrors("???")
        publishMessages("???")
        sleep(0.2)
    global ready
    ready = False #arduino is not ready
    while 1:
        try: #try to attach arduino
            arduino=serial.Serial("/dev/ttyACM0",9600)
            break
        except:
            #send error
            publishErrors("ATMega has not been attached")
            sleep(0.5) #wait for next attempt
    ready = True #now it's ready
    tic = timeit.default_timer() #set the timer
    while not rospy.is_shutdown() and arduino is not None:
        try: #try to read from arduino
            if arduino and arduino.in_waiting: #if there is something
                tic = timeit.default_timer() #reset timer
                temp=arduino.readline().decode("utf-8") #read
                if "!!" in temp: #if it is an error
                    publishErrors( temp.replace('!!', '') ) #send over errors topic
                else: #else, over sensors topic
                    publishSensors(temp)
        except: #it it couldn't read from arduino, then it has to be attached again
            ready = False #arduino is not ready
            while 1:
                try: #try to attach
                    arduino=serial.Serial("/dev/ttyACM0",9600)
                    break
                except: #publish error and wait
                    publishErrors("ATMega has not been attached")
                    sleep(0.5)
            ready = True #now it's ready
        if timeit.default_timer()-tic >= 4: #if it didn't respond for more than 4 seconds
            tic = timeit.default_timer()
            publishErrors("AtMega is not responding") #send an error message

if __name__ == '__main__':
    main()
