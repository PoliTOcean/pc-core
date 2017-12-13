# -*- coding: utf-8 -*-

'''This python script has the main core of ROS communications from GUI to ROV.
It receives:
 - sensors data => print on sensors widget
 - messages => print on console
 - errors => print on console (in red)
 - cameras => print over cams widgets
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

class ROV:
    def __init__(self, win):
        self.window = win
        self.zeroPressure = 0
        self.received = False
        #cameras subscribers
        self.image0_sub = rospy.Subscriber("/webcam/image_raw", Image, self.camerasCallback, 0)
        self.image1_sub = rospy.Subscriber("/webcam/image_raw", Image, self.camerasCallback, 1)
        self.image2_sub = rospy.Subscriber("/webcam/image_raw", Image, self.camerasCallback, 2)
        #errors, messages and sensors subscribers
        self.errors_sub = rospy.Subscriber("errors", String, self.errorsCallback)
        self.sensors_sub = rospy.Subscriber("sensors", sensors_data, self.sensorsCallback)
        self.messages_sub = rospy.Subscriber("messages", String, self.messagesCallback)
        #commands publisher
        self.commands_pub = rospy.Publisher("commands", String, queue_size=3)
        self.toCalibrate = False
        self.firstCalibrationDone = False
        self.zeroDepthPressure = 0.0
        self.zeroPitch = 0.0
        self.zeroRoll = 0.0

    #send command over the topic
    def sendCommand(self, comm):
        #write comm into the "commands" Topic
        try:
            if "calib" in str(comm).lower():
                self.toCalibrate = True
            else:
                self.commands_pub.publish( str(comm) ) #send command
        except Exception as e:
            publishErrors('GUI_ROV', "Commands topic publisher: "+str(e))
        #check here for "arm" commands and eventually modify its image on Window

    #print sensors over widgets
    def sensorsCallback(self, data):
        self.received = True
        #get values from "sensors" Topic
        if self.toCalibrate:
            self.calibrate(data.pressure, data.pitch, data.roll)
        if self.firstCalibrationDone:
            depth = (data.pressure-self.zeroDepthPressure)/1000
            self.window.update_sensors(depth, data.temperature, data.pitch, data.roll)

    #calibration function
    def calibrate(self, pressure, pitch, roll):
        self.firstCalibrationDone = True
        self.toCalibrate = False
        self.zeroDepthPressure = pressure
        self.zeroPitch = pitch
        self.zeroRoll = roll

    #print messages on the console
    def messagesCallback(self, data):
        self.received = True
        if "???" not in str(data):
            self.window.updateConsole(str(data).replace('data: ', ''), 1) #print on console

    #print errors over console (in RED)
    def errorsCallback(self, data):
        self.received = True
        #get from "errors" Topic and print them into the Console widget (in RED)
        if "???" not in str(data):
            self.window.updateConsole("!!! "+str(data).replace('data: ', '')+" !!!\n", 2) #print on console in RED

    #paint images over cameras widgets
    def camerasCallback(self, img, index):
        self.received = True
        #get from "cameras" Topic and print on the video
        self.window.setCurrentFrame(index, img)
        #DEBUG PRINT
        #print("I'm converting")

    #check if it received something to tell if ROV is awake
    def isAwake(self):
        if self.received:
            self.received = False
            return True
        return False
