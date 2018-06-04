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
import datetime
import re

#take ROS path from bash environment
PATH_ROS = os.environ['ROS_PACKAGE_PATH'].split(':')[0]
form_class = uic.loadUiType(PATH_ROS+"/politocean/scripts/gui/design_fhd.ui")[0]

#main window class
class Window(QtGui.QMainWindow,form_class):
	#init of Main Window

    def __init__(self,parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.setWindowTitle('PoliTOcean')

        #we need this variable to send commands from here
        self.rov = None

        #set default sensors values
        self.depth_value = -1.11
        self.pitch_value = 10
        self.roll_value = 10
        self.temperature_value = 34

        #bridge for image converting
        self.bridge = CvBridge()

        #logo
        self.ppm = QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/politocean3.png")
        self.politocean.setPixmap( self.ppm.scaled(270, 45, QtCore.Qt.KeepAspectRatio) )
        self.politocean.show()

        #loading image of Arm Widget
         #state 0 axis 0 nipper 0 (x,open)
         #state 1 axis 0 nipper 1 (x,close)
         #state 2 axis 1 nipper 0 (y,open)
         #state 3 axis 1 nipper 1 (y,close)
#deprecated ARM WIDGET
        #self.stateArm = []
        #self.stateArm.append(QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/arm1.png"))
        #self.stateArm.append(QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/arm2.png"))
        #self.stateArm.append(QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/politocean3.png"))
        #self.stateArm.append(QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/politocean3.png"))


        #import of style sheet (written in CSS)
        styleSheet = open(PATH_ROS+"/politocean/scripts/gui/style.css", "r")
        self.ROVdata.setStyleSheet( styleSheet.read() )

        #create image for absent signal
        segn_ass = cv2.imread(PATH_ROS+"/politocean/scripts/gui/segnale_assente.png")
        height, width, bpc = segn_ass.shape
        self.QTsegn_ass = QtGui.QImage(segn_ass.data, width, height, bpc*width, QtGui.QImage.Format_RGB888)
        #set as initial frame
        self.frm = [self.QTsegn_ass, self.QTsegn_ass, self.QTsegn_ass]

        #creating status images for the components
        self.vpm = QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/V.png").scaled(17,17)
        self.xpm = QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/X.png").scaled(17,17)
        self.wpm = QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/wait.png").scaled(17,17)
        self.joystickStatus.setPixmap( self.xpm ) #set "disabled" image at first
        self.ATMegaStatus.setPixmap( self.xpm )

        #connect button pression to relative functions
        self.startVideoBtn.clicked.connect(self.startVideo_clicked)
        self.calibBtn.clicked.connect(self.calibrate_clicked)
        self.startBtn.clicked.connect(self.start_clicked)
        self.stopBtn.clicked.connect(self.stop_clicked)
        self.sendBtn.clicked.connect(self.send_clicked)
        self.clearBtn.clicked.connect(self.initConsole)
        #set console read only
        self.console.setReadOnly(True)

        #install event filters
        self.cmdInput.installEventFilter(self)
        self.installEventFilter(self)

        #boolean for start the connection
        self.running = False

        #set default cam indexes
        #self.cam = [0, 1, 2]
        self.frame1 = None
        self.frame2 = None

        #flag
        self.altPressed = False
        self.cmdFocus = False

        #set a timer to call setFrame function (ca 29fps)
        self.timer = QTimer(self)
        self.connect(self.timer, SIGNAL("timeout()"), self.setFrame)
        self.timer.start(34)

        #set a timer to call sensorsUpdate function (10Hz)
        self.timer2 = QTimer(self)
        self.connect(self.timer2, SIGNAL("timeout()"), self.sensorsUpdate)
        self.timer2.start(100)

        #connect signals to update
        self.connect(self, SIGNAL("updateHTML()"), self.updateConsoleHTML)
        self.connect(self, SIGNAL("updateATMega(int)"), self.ATMegaEnabled)
        self.connect(self, SIGNAL("updateJoystick(int)"), self.joystickEnabled)
        #self.connect(self, SIGNAL("updateArm()"), self.armUpdate)

        #set the path of console log file
        self.consolePath = PATH_ROS+"/politocean/scripts/gui/console.log"
        self.initConsole()

        #connect checkboxes to the console
        self.messages.stateChanged.connect(lambda:self.updateConsole("", TYPE.UPDATE))
        self.errors.stateChanged.connect(lambda:self.updateConsole("", TYPE.UPDATE))
        self.commands.stateChanged.connect(lambda:self.updateConsole("", TYPE.UPDATE))

    # update arm variable
    def setArmAxis(self,axis):
        self.axis = axis
        self.emit(SIGNAL("updateArm()"))

    def setArmNipper(self,nipper):
        self.nipper = nipper
        self.emit(SIGNAL("updateArm()"))

    #update sensorsWidget
    def sensorsUpdate(self):
        self.depth.display(float(str("{0:.3f}".format(self.depth_value))))
        self.temp.display(float(str("{0:.3f}".format(self.temperature_value))))
        self.pitch.display(float(str("{0:.3f}".format(self.pitch_value))))
        self.roll.display(float(str("{0:.3f}".format(self.roll_value))))

    #update armWidget (DEPRECATED)
    #def armUpdate(self):
    #    if self.axis >= 1 and self.nipper == 0:
    #        self.label_5.setPixmap(self.stateArm[0].scaled(400, 100, QtCore.Qt.KeepAspectRatio) )
    #        self.label_5.show()
    #    if self.axis >= 1 and self.nipper == 1:
    #       self.label_5.setPixmap(self.stateArm[1].scaled(400, 100, QtCore.Qt.KeepAspectRatio) )
    #       self.label_5.show()
    #    if self.axis <= -1 and self.nipper == 0:
    #        self.label_5.setPixmap(self.stateArm[2].scaled(400, 100, QtCore.Qt.KeepAspectRatio) )
    #        self.label_5.show()
    #    if self.axis <= -1 and self.nipper == 1:
    #        self.label_5.setPixmap(self.stateArm[3].scaled(400, 100, QtCore.Qt.KeepAspectRatio) )
    #        self.label_5.show()


    #set the ROV variable
    def setROV(self, rov):
        self.rov = rov

    #parse event function
    def eventFilter(self, widget, event):
        if widget is self.cmdInput: #if the widget is the command input line
            if event.type()==QtCore.QEvent.KeyPress: #and a key has been pressed
                k = event.key()
                if k == QtCore.Qt.Key_Return: #check if it's ENTER
                    self.send_clicked()         #send command
                elif k == QtCore.Qt.Key_Escape: #elif it's ESC
                    self.setFocus()             #focus the window
                    self.cmdFocus = True    #flag to stop ESC propagation (see below)
        elif widget is self and event.type()==QtCore.QEvent.KeyPress:    #else if it's on the window
            k = event.key()
            
            if k == QtCore.Qt.Key_C: #if it's C
                self.calibrate_clicked()    #calibrate
            elif k == QtCore.Qt.Key_V: #if it's V
                self.startVideo_clicked()   #video start/stop
            elif k == QtCore.Qt.Key_S: #if it's S
                self.stop_clicked()         #STOP to ROV
            elif k == QtCore.Qt.Key_G: #if it's G
                self.start_clicked()        #START to ROV
            elif k == QtCore.Qt.Key_Escape: #if it's ESC
                if not self.cmdFocus:   #check the cmdFocus flag (see above) to avoid propagation
                    self.initConsole()          #re-init console (clear)
                self.cmdFocus = False   #reset the flag
            elif k == QtCore.Qt.Key_M: #if it's M
                self.messages.nextCheckState() #change messages checkbox state
            elif k == QtCore.Qt.Key_E: #if it's E
                self.errors.nextCheckState() #change errors checkbox state
            elif k == QtCore.Qt.Key_X: #if it's X
                self.commands.nextCheckState() #change commands checkbox state
            elif k == QtCore.Qt.Key_Control or k == QtCore.Qt.Key_Alt: #if it's CTRL or ALT
                self.altPressed = not self.altPressed   #toggle altPressed flag
                self.showShortcuts()    #show shortcuts (only if the flag is True)
            elif k == QtCore.Qt.Key_Backslash:  #if it's \
                self.cmdInput.setFocus()    #focus command line input
            elif k == QtCore.Qt.Key_F1: #if it's F1
                self.tab.setCurrentIndex(0) #select 1st tab
            elif k == QtCore.Qt.Key_F2: #if it's F2
                self.tab.setCurrentIndex(1) #select 2nd tab
            elif k == QtCore.Qt.Key_F3: #if it's F3
                self.tab.setCurrentIndex(2) #select 3rd tab
            elif k == QtCore.Qt.Key_F4: #if it's F4
                self.tab.setCurrentIndex(3) #select 4th tab                         #and cameras are running
            
        return QtGui.QWidget.eventFilter(self, widget, event)

    #function to show keyboard shortcuts
    def showShortcuts(self):
        #check the altPressed flag and show/hide shortcuts help
        if self.altPressed:
            self.startVideoBtn.setText( self.startVideoBtn.text()+' [V]' )
            self.calibBtn.setText( self.calibBtn.text()+' [C]' )
            self.startBtn.setText( self.startBtn.text()+' [G]' )
            self.stopBtn.setText( self.stopBtn.text()+' [S]' )
            self.clearBtn.setText( self.clearBtn.text()+' [esc]' )
            self.sendBtn.setText( self.sendBtn.text()+' [Enter]' )
            self.messages.setText( self.messages.text()+' [M]' )
            self.errors.setText( self.errors.text()+' [E]' )
            self.commands.setText( self.commands.text()+' [X]' )
            self.camRadio1.setText( '[1] '+self.camRadio1.text() )
            self.camRadio2.setText( '[2] '+self.camRadio2.text() )
            self.camRadio3.setText( '[3] '+self.camRadio3.text() )
            self.tab.setTabText(0,  self.tab.tabText(0)+' [F1]' )
            self.tab.setTabText(1,  self.tab.tabText(1)+' [F2]' )
            self.tab.setTabText(2,  self.tab.tabText(2)+' [F3]' )
            self.tab.setTabText(3,  self.tab.tabText(3)+' [F4]' )
            self.recognize.setText( self.recognize.text()+' [R]' )
            self.showPlot.setText( self.showPlot.text()+' [P]' )
        else:
            self.startVideoBtn.setText( self.startVideoBtn.text().replace(' [V]', '') )
            self.calibBtn.setText( self.calibBtn.text().replace(' [C]', '') )
            self.startBtn.setText( self.startBtn.text().replace(' [G]', '') )
            self.stopBtn.setText( self.stopBtn.text().replace(' [S]', '') )
            self.clearBtn.setText( self.clearBtn.text().replace(' [esc]', '') )
            self.sendBtn.setText( self.sendBtn.text().replace(' [Enter]', '') )
            self.messages.setText( self.messages.text().replace(' [M]', '') )
            self.errors.setText( self.errors.text().replace(' [E]', '') )
            self.commands.setText( self.commands.text().replace(' [X]', '') )
            self.camRadio1.setText( self.camRadio1.text().replace('[1] ', '') )
            self.camRadio2.setText( self.camRadio2.text().replace('[2] ', '') )
            self.camRadio3.setText( self.camRadio3.text().replace('[3] ', '') )
            self.tab.setTabText(0, self.tab.tabText(0).replace(' [F1]', '') )
            self.tab.setTabText(1, self.tab.tabText(1).replace(' [F2]', '') )
            self.tab.setTabText(2, self.tab.tabText(2).replace(' [F3]', '') )
            self.tab.setTabText(3, self.tab.tabText(3).replace(' [F4]', '') )
            self.recognize.setText( self.recognize.text().replace(' [R]', '') )
            self.showPlot.setText( self.showPlot.text().replace(' [P]', '') )


    #start ROV function
    def start_clicked(self):
        self.updateConsole("thumb2", TYPE.COMMAND)
        self.rov.enable12Volt()

    #stop ROV function
    def stop_clicked(self):
        self.updateConsole("thumb", TYPE.COMMAND)
        self.rov.disable12Volt()

    #calibrate function
    def calibrate_clicked(self):
        self.updateConsole("calib", TYPE.COMMAND)
        self.rov.sendCommand("calib")

    #send commands from command input
    def send_clicked(self):
        self.updateConsole(self.cmdInput.text(), TYPE.COMMAND)
        self.rov.sendCommand(self.cmdInput.text())
        self.cmdInput.setText("")

    #toggle running state
    def startVideo_clicked(self):
        if not self.running:
            #start of connection
            self.running = True;
            text = 'Stop video'
            if self.altPressed:
                text+=' [V]'
            self.startVideoBtn.setText(text)
        else:
            #end of connection
            text = 'Start video'
            if self.altPressed:
                text+=' [V]'
            self.startVideoBtn.setText(text)
            self.running = False;
            self.frm = [self.QTsegn_ass, self.QTsegn_ass, self.QTsegn_ass]

    #set img as current frame of cam[index]
    def setCurrentFrame(self, index, img):
        if not self.running:
            return
        #i = self.cam[index]
        width = 640
        height = 480
        #parse image to get a QImage object
        #self.frm[i] = self.parse_image(img, width, height)
        if index == 0:
            self.frame1 = img
        else:
            self.frame2 = img

    #set self.frm[] to all the cams
    def setFrame(self):
        if not self.running:
            return
        
        self.update_frame_cam(self.mainCam, self.frame1)
        self.update_frame_cam(self.cam3, self.frame2)

    #create pixmap and set it on the cam
    def update_frame_cam(self, cam, frm):
        width = 640
        height = 480
        frm = self.parse_image(frm, width, height)
        pixmap = QPixmap.fromImage(frm)
        cam.setPixmap( pixmap )

    #method to take an Image object and generate a QImage one
    def parse_image(self, data, width, height):
        if data is None:
            return self.QTsegn_ass
        img = None
        try:
            img = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)
            return None
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)	    
        height, width, bpc = img.shape
        bpl = bpc * width
        image = QtGui.QImage(img.data, width, height, bpl, QtGui.QImage.Format_RGB888)

        return image

    #init console (even clear it)
    def initConsole(self):
        consoleFile = open(self.consolePath, 'w')
        consoleFile.write("")
        consoleFile.close()
        self.updateConsole("", 4)

    #append data to console
    def updateConsole(self, data, dataType):
        #check if data is empty and exit, but only if we are not updating
        if str(data).strip()=="" and dataType!=TYPE.UPDATE:
            return #return

        data = str(data).decode("utf-8")

        #check which type of message it is
        consoleFile = open(self.consolePath, "a")
        if dataType==TYPE.COMMAND: #commands
            consoleFile.write(("#C"+data+"#-#").decode("utf-8"))
        elif dataType==TYPE.MESSAGE: #messages (color BLUE)
            consoleFile.write(("#M"+data+"#-#").decode("utf-8"))
        elif dataType==TYPE.ERROR: #errors (color RED)
            consoleFile.write(("#E"+data+"#-#").decode("utf-8"))
        consoleFile.close()

        readConsole = open(self.consolePath, 'r')
        fileRead = readConsole.read()

        #check the checkboxes state in order to select the right type of messages
        #that has to be printed on the console
        if not self.commands.isChecked():
            fileRead = re.sub('#C.*?#-#', '', fileRead)
        if not self.errors.isChecked():
            fileRead = re.sub('#E.*?#-#', '', fileRead)
        if not self.messages.isChecked():
            fileRead = re.sub('#M.*?#-#', '', fileRead)

        #replace the custom notation with HTML
        fileRead = fileRead.replace("#M", "<p style=\"color: #00f\">   ") #blue text
        fileRead = fileRead.replace("#E", "<p style=\"color: #f00\">   ") #red text
        fileRead = fileRead.replace("#C", "<p> > ") #plain text
        fileRead = fileRead.replace("#-#", "</p>#-#")

        #split all lines
        fileR = fileRead.split("#-#")

        #then join them selecting the first 20
        html = ""
        if len(fileRead)>20:
            html = "".join(fileR[-20:])
        else:
            html = "".join(fileR)
        #add top message
        html = "<p align=\"center\" style=\"font-weight: bold\"> \
                        ------------------------  \
                        Welcome to the PoliTOcean GUI \
                        ------------------------  \
                </p>"+html
        self.QtHTML = QString(html) #transform it in a QString object

        #emit signal to update the console
        self.emit(SIGNAL("updateHTML()"))

    #set new HTML on the console
    def updateConsoleHTML(self):
        self.console.setHtml( self.QtHTML ) #append to console
        self.console.moveCursor(QTextCursor.End) #set cursor to end to follow lines

    #set ATMega status (change image)
    def ATMegaEnabled(self, status):
        if status==STATUS.ENABLED:
            self.ATMegaStatus.setPixmap( self.vpm )
            self.ATMega.setText("ATMega connected")
        elif status==STATUS.DISABLED:
            self.ATMegaStatus.setPixmap( self.xpm )
            self.ATMega.setText("ATMega disconnected")
        elif status==STATUS.BUSY:
            self.ATMegaStatus.setPixmap( self.wpm )
            self.ATMega.setText("ATMega busy")
        self.ATMegaStatus.show()

    #set Joystick status (change image)
    def joystickEnabled(self, status):
        if status==STATUS.ENABLED:
            self.joystickStatus.setPixmap( self.vpm )
            self.joystick.setText("Joystick connected")
        elif status==STATUS.DISABLED:
            self.joystickStatus.setPixmap( self.xpm )
            self.joystick.setText("Joystick disconnected")

    #set the value of sensors
    def setSensorsValue(self,depth,pitch,roll,temperature):
        self.depth_value = depth
        self.pitch_value = pitch
        self.roll_value = roll
        self.temperature_value = temperature

    #get running state
    def isRunning(self):
        return self.running
