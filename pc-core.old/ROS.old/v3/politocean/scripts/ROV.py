# -*- coding: utf-8 -*-

'''This python script has the main core of ROS communications from GUI to ROV.
It receives:
 - sensors data => print on sensors widgets
 - messages => print on console (in blue)
 - errors => print on console (in red)
 - cameras => print over cams widgets
 - components => change components state images
It sends:
 - commands => from console
 '''
import roslib
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from time import sleep
from politocean.msg import *
from errmess_publisher import *
from PyQt4.QtCore import *

#ROV class
class ROV:
    def __init__(self, win):
        self.window = win
        self.zeroPressure = 0
        self.received = False
        #cameras subscribers
        self.image0_sub = rospy.Subscriber("/Cam0/webcam/image_raw", Image, self.camerasCallback, 0)
        self.image1_sub = rospy.Subscriber("/Cam1/webcam/image_raw", Image, self.camerasCallback, 1)
        self.image2_sub = rospy.Subscriber("/Cam2/webcam/image_raw", Image, self.camerasCallback, 2)
        #errors, messages, sensors and components subscribers
        errors_sub = rospy.Subscriber("errors", String, self.errorsCallback)
        sensors_sub = rospy.Subscriber("sensors", sensors_data, self.sensorsCallback)
        messages_sub = rospy.Subscriber("messages", String, self.messagesCallback)
        comp_sub = rospy.Subscriber("components", component_data, self.componentsCallback)

        #commands publisher
        self.commands_pub = rospy.Publisher("commands", String, queue_size=3)

        #set "global" variables
        self.toCalibrate = False
        self.firstCalibrationDone = False
        self.zeroDepthPressure = 0.0
        self.zeroPitch = 0.0
        self.zeroRoll = 0.0

    #callback function for components updates
    def componentsCallback(self, data):
        if data.status==STATUS.REQUEST: #if it's just a request
            return #exit
        if data.ID==ID.ATMEGA: #check for the component
            #and send the signal with the status
            self.window.emit(SIGNAL("updateATMega(int)"), data.status)
        elif data.ID==ID.JOYSTICK:
            self.window.emit(SIGNAL("updateJoystick(int)"), data.status)

    #print sensors over widgets
    def sensorsCallback(self, data):
        self.received = True
        depth = -1.111
        if self.toCalibrate: #if it has to be calibrated (calib command)
            self.calibrate(data.pressure, data.pitch, data.roll) #calibrate
        if self.firstCalibrationDone: #if it's already been calibrated
            depth = (data.pressure-self.zeroDepthPressure)/1000 #calculate depth in meters
        self.window.update_sensors(depth, data.pitch, data.roll) #update sensors on the gui

    #print messages on the console
    def messagesCallback(self, data):
        self.received = True
        if "???" not in str(data):
            self.window.updateConsole(str(data).replace('data: ', ''), TYPE.MESSAGE) #print on console

    #print errors over console (in RED)
    def errorsCallback(self, data):
        self.received = True
        #get from "errors" Topic and print them into the Console widget (in RED)
        if "???" not in str(data):
            self.window.updateConsole("!!! "+str(data).replace('data: ', '')+" !!!", TYPE.ERROR)

    #paint images over cameras widgets
    def camerasCallback(self, img, index):
        self.received = True
        #get from "cameras" Topic and print on the video
        self.window.setCurrentFrame(index, img)
        #DEBUG PRINT
        #print("I'm converting")


    #send command over the topic
    def sendCommand(self, comm):
        #write comm into the "commands" Topic
        try:
            if "calib" in str(comm).lower():
                self.toCalibrate = True
            elif str(comm)!="":
                self.commands_pub.publish( str(comm) ) #send command
        except Exception as e:
            publishErrors(NODE.GUI, "Commands topic publisher: "+str(e))
        #check here for "arm" commands and eventually modify its image on Window

    #calibration function
    def calibrate(self, pressure, pitch, roll):
        self.firstCalibrationDone = True
        self.toCalibrate = False
        self.zeroDepthPressure = pressure
        self.zeroPitch = pitch
        self.zeroRoll = roll

    #check if it received something to tell if ROV is awake
    def isAwake(self):
        if self.received:
            self.received = False
            return True
        return False
