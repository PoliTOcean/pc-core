# -*- coding: utf-8 -*-

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import timeit
from time import sleep
from errmess_publisher import *

#take ROS path from environment
PATH = os.environ['ROS_PACKAGE_PATH']
PATH = PATH.split(':')
PATH_ROS = PATH[0]
form_class = uic.loadUiType(PATH_ROS+"/politocean/scripts/gui/simple.ui")[0]

class Window(QtGui.QMainWindow,form_class):

	#init of Main Window
    def __init__(self,parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.setWindowTitle('PolitOcean')

        #bridge for image converting
        self.bridge = CvBridge()

        #logo
        self.politocean.resize(400,150)
        self.politocean.setPixmap(QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/politocean3.png"))
        self.politocean.show()

        #create image for absent signal
        segn_ass = cv2.imread(PATH_ROS+"/politocean/scripts/gui/segnale_assente.png")
        height, width, bpc = segn_ass.shape
        self.QTsegn_ass = QtGui.QImage(segn_ass.data, width, height, bpc*width, QtGui.QImage.Format_RGB888)
        #set as initial frame
        self.frm = [self.QTsegn_ass, self.QTsegn_ass, self.QTsegn_ass]

        #the connect button pression to relative functions
        self.startVideoBtn.clicked.connect(self.startVideo_clicked)
        self.calibBtn.clicked.connect(self.calibrate_clicked)
        self.startBtn.clicked.connect(self.start_clicked)
        self.stopBtn.clicked.connect(self.stop_clicked)
        self.sendBtn.clicked.connect(self.send_clicked)
        self.clearBtn.clicked.connect(self.console.clear)
        #set console read only
        self.console.setReadOnly(True)

        #install event filters
        self.cmdInput.installEventFilter(self)
        self.cam2.installEventFilter(self)
        self.cam3.installEventFilter(self)
        self.installEventFilter(self)

        #boolean for start the connection
        self.running = False

        #set default cam indexes
        self.cam = [0, 1, 2]

        #commands (it will be needed for the console)
        self.command = ""

        #set a timer to call setFrame function
        timer = QTimer(self)
        self.connect(timer, SIGNAL("timeout()"), self.setFrame)
        timer.start(33)

    #parse event function
    def eventFilter(self, widget, event):
        if event.type()==QtCore.QEvent.KeyPress: #if a key is pressed
            if widget is self.cmdInput: #if it's on the command input bar
                if event.key() == QtCore.Qt.Key_Return: #end it's ENTER
                    self.send_clicked()                     #send command
                elif event.key() == QtCore.Qt.Key_Escape: #elif it's ESC
                    self.cmdInput.setText("")               #clear command input
            elif widget is self:    #else if it's on the window
                if event.key() == QtCore.Qt.Key_1: #if it's 1, 2 or 3
                    self.setMainCamera(0)           #set main camera
                elif event.key() == QtCore.Qt.Key_2: #if it's 2
                    self.setMainCamera(1)
                elif event.key() == QtCore.Qt.Key_3:
                    self.setMainCamera(2)
                elif event.key() == QtCore.Qt.Key_C: #if it's C
                    self.calibrate_clicked()    #calibrate
                elif event.key() == QtCore.Qt.Key_V: #if it's V
                    self.startVideo_clicked()   #video start/stop
                elif event.key() == QtCore.Qt.Key_S: #if it's S
                    self.stop_clicked()         #STOP to ROV
                elif event.key() == QtCore.Qt.Key_G: #if it's G
                    self.start_clicked()        #START to ROV
                elif event.key() == QtCore.Qt.Key_Escape: #if it's ESC
                    self.console.clear()        #clear the console
                elif event.key() == QtCore.Qt.Key_M: #if it's M
                    self.messages.nextCheckState() #change messages checkbox state
                elif event.key() == QtCore.Qt.Key_E: #if it's E
                    self.errors.nextCheckState() #change errors checkbox state
                elif event.key() == QtCore.Qt.Key_X: #if it's X
                    self.commands.nextCheckState() #change commands checkbox state
        elif (event.type()==QtCore.QEvent.MouseButtonPress #else if it's a mouse event
                and event.button() == QtCore.Qt.LeftButton): #and it's left button
            ind = 0
            if widget is self.cam2: #check for which cam has been pressed
                ind = 1
            elif widget is self.cam3:
                ind = 2
            j = 0
            for i in range(1,3): #let's see which cam is pointing to the "ind" cam widget
                if self.cam[i]==ind:
                    j = i
            self.setMainCamera(j) #set main camera
        return QtGui.QWidget.eventFilter(self, widget, event)

    #set camInd as main camera
    def setMainCamera(self, camInd):
        j0 = 0
        for i in range(1,3): #let's see which camera is the main one
            if self.cam[i]==0:
                j0 = i
        #swap pointers
        self.cam[j0] = self.cam[camInd]
        self.cam[camInd] = 0

    #get commands from console
    def getCommands(self):
        comm = self.command
        self.command = ""
        return comm

    #set img as current frame of cam[index]
    def setCurrentFrame(self, index, img):
        if not self.running:
            return
        i = self.cam[index]
        if i==0:
            width = self.mainCam.frameSize().width()
            height = self.mainCam.frameSize().height()
        else:
            width = self.cam2.frameSize().width()
            height = self.cam2.frameSize().height()
        #parse image to get a QImage object
        self.frm[i] = self.parse_image(img, width, height)

    #start ROV function
    def start_clicked(self):
        self.command = "G"
        self.updateConsole(self.command, 0)

    #stop ROV function
    def stop_clicked(self):
        self.command = "SSS"
        self.updateConsole(self.command, 0)

    #calibrate function
    def calibrate_clicked(self):
        self.command = "calib"
        self.updateConsole(self.command, 0)

    #append data to console
    def updateConsole(self, data, dataType):
        if str(data).strip()=="": #if it's empty
            return #return

        line = "<span" #init the line
        #check which type of message it is
        if dataType==0: #commands
            if not self.commands.isChecked():
                return
            line += "> > "+str(data)
        elif dataType==1: #messages (color BLUE)
            if not self.messages.isChecked():
                return
            line += " style=\"color:#0000ff\">   "+str(data)
        elif dataType==2: #errors (color RED)
            if not self.errors.isChecked():
                return
            line += " style=\"color:#ff0000\">   "+str(data)
        line+="</span><br>" #close line
        self.console.insertHtml( QString(line) ) #append to console
        self.console.moveCursor(QTextCursor.End) #set cursor to end to follow lines

    #toggle running state
    def startVideo_clicked(self):
        if not self.running:
			#start of connection
            self.running = True;
            self.startVideoBtn.setText('Stop video')
            sleep(0.1)
        else:
        	#end of connection
            self.startVideoBtn.setText('Start video')
            self.running = False;
            sleep(0.1)
            self.frm = [self.QTsegn_ass, self.QTsegn_ass, self.QTsegn_ass]

    #send commands from command input
    def send_clicked(self):
        self.command = self.cmdInput.text()
        self.cmdInput.setText("")
        self.updateConsole(self.command, 0)

    #update sensors widget
    def update_sensors(self, depth, temp, pitch, roll):
        self.depth.setText((str(depth)+"m").decode("utf-8"))
        self.temperature.setText((str(temp)+"°C").decode("utf-8"))
        self.pitch.setText((str(pitch)+"°").decode("utf-8"))
        self.roll.setText((str(roll)+"°").decode("utf-8"))

    #get running state
    def isRunning(self):
        return self.running

    #set self.frm[] to all the cams
    def setFrame(self):
        if not self.running:
            return
        self.update_frame_cam(self.mainCam, self.frm[0])
        self.update_frame_cam(self.cam2, self.frm[1])
        self.update_frame_cam(self.cam3, self.frm[2])

    #create pixmap and set it on the cam
    def update_frame_cam(self, cam, frm):
        pixmap = QPixmap.fromImage(frm)
        cam.setPixmap( pixmap )

    #method to take an Image object and generate a QImage one
    def parse_image(self, data, width, height):
        if data is None:
            return self.QTsegn_ass
        img = None
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return None
        img_height, img_width, img_colors = img.shape
        scale_w = float(width) / float(img_width)
        scale_h = float(height) / float(img_height)
        scale = min([scale_w, scale_h])

        #if scale == 0:
        #scale = 1
        #if scale2 == 0:
        #scale2 = 1

        img = cv2.resize(img, None, fx=scale_w, fy=scale, interpolation = cv2.INTER_CUBIC)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        height, width, bpc = img.shape
        bpl = bpc * width
        image = QtGui.QImage(img.data, width, height, bpl, QtGui.QImage.Format_RGB888)

        return image
