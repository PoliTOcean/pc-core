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
form_class = uic.loadUiType(PATH_ROS+"/politocean/scripts/gui/simple.ui")[0]

#main window class
class Window(QtGui.QMainWindow,form_class):
	#init of Main Window
    def __init__(self,parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.setWindowTitle('PoliTOcean')

        #bridge for image converting
        self.bridge = CvBridge()

        #logo
        ppm = QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/politocean3.png")
        self.politocean.setPixmap( ppm.scaled(270, 45, QtCore.Qt.KeepAspectRatio) )
        self.politocean.show()

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
        self.cam2.installEventFilter(self)
        self.cam3.installEventFilter(self)
        self.installEventFilter(self)

        #connect radio toggle to the function
        self.camRadio1.toggled.connect(self.toggledRadio)
        self.camRadio2.toggled.connect(self.toggledRadio)
        self.camRadio3.toggled.connect(self.toggledRadio)

        #boolean for start the connection
        self.running = False

        #set default cam indexes
        self.cam = [0, 1, 2]

        #it will be needed for the console
        self.command = ""

        #flag
        self.altPressed = False
        self.cmdFocus = False

        #set a timer to call setFrame function
        timer = QTimer(self)
        self.connect(timer, SIGNAL("timeout()"), self.setFrame)
        timer.start(45)

        #connect signals to update
        self.connect(self, SIGNAL("updateHTML()"), self.updateConsoleHTML)
        self.connect(self, SIGNAL("updateATMega(int)"), self.ATMegaEnabled)
        self.connect(self, SIGNAL("updateJoystick(int)"), self.joystickEnabled)

        #set the path of console log file
        self.consolePath = PATH_ROS+"/politocean/scripts/gui/console.log"
        self.initConsole()

        #connect checkboxes to the console
        self.messages.stateChanged.connect(lambda:self.updateConsole("", TYPE.UPDATE))
        self.errors.stateChanged.connect(lambda:self.updateConsole("", TYPE.UPDATE))
        self.commands.stateChanged.connect(lambda:self.updateConsole("", TYPE.UPDATE))

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
            if k == QtCore.Qt.Key_1: #if it's 1, 2 or 3
                self.camRadio1.setChecked(True)           #set main camera
            elif k == QtCore.Qt.Key_2: #if it's 2
                self.camRadio2.setChecked(True)
            elif k == QtCore.Qt.Key_3:
                self.camRadio3.setChecked(True)
            elif k == QtCore.Qt.Key_C: #if it's C
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
                self.tab.setCurrentIndex(3) #select 4th tab
        elif (event.type()==QtCore.QEvent.MouseButtonPress  #else if it's a mouse event
                and event.button() == QtCore.Qt.LeftButton  #and it's left button
                and not widget is self.mainCam              #and is not the main camera
                and self.running):                          #and cameras are running
            ind = 0
            if widget is self.cam2: #check for which cam has been pressed
                ind = 1
            elif widget is self.cam3:
                ind = 2
            j = 0
            for i in range(1,3): #let's see which cam is pointing to the "ind" cam widget
                if self.cam[i]==ind:
                    j = i
            if j==0:    #check which camera was pointing to the clicked widget
                self.camRadio1.setChecked(True) #check the right radio (this will call the function to update cameras)
            elif j==1:
                self.camRadio2.setChecked(True)
            elif j==2:
                self.camRadio3.setChecked(True)
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

    #function called when a radio has been toggled
    def toggledRadio(self):
        #check which radio is checked
        if self.camRadio1.isChecked():
            self.setMainCamera(0) #and call setMainCamera with the right camera index
        elif self.camRadio2.isChecked():
            self.setMainCamera(1)
        elif self.camRadio3.isChecked():
            self.setMainCamera(2)

    #start ROV function
    def start_clicked(self):
        self.command = "G"
        self.updateConsole(self.command, TYPE.COMMAND)

    #stop ROV function
    def stop_clicked(self):
        self.command = "SSS"
        self.updateConsole(self.command, TYPE.COMMAND)

    #calibrate function
    def calibrate_clicked(self):
        self.command = "calib"
        self.updateConsole(self.command, TYPE.COMMAND)

    #send commands from command input
    def send_clicked(self):
        self.command = self.cmdInput.text()
        self.cmdInput.setText("")
        self.updateConsole(self.command, TYPE.COMMAND)

    #toggle running state
    def startVideo_clicked(self):
        if not self.running:
            #start of connection
            self.running = True;
            text = 'Stop video'
            if self.altPressed:
                text+=' [V]'
            self.startVideoBtn.setText(text)
            sleep(0.1)
        else:
            #end of connection
            text = 'Start video'
            if self.altPressed:
                text+=' [V]'
            self.startVideoBtn.setText(text)
            self.running = False;
            sleep(0.1)
            self.frm = [self.QTsegn_ass, self.QTsegn_ass, self.QTsegn_ass]

    #set camInd as main camera
    def setMainCamera(self, camInd):
        j0 = 0
        for i in range(1,3): #let's see which camera is the main one
            if self.cam[i]==0:
                j0 = i
        #swap pointers
        self.cam[j0] = self.cam[camInd]
        self.cam[camInd] = 0

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

        img = cv2.resize(img, None, fx=scale_w, fy=scale, interpolation = cv2.INTER_CUBIC)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
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
            xpm = QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/X.png").scaled(17,17)
            self.ATMegaStatus.setPixmap( self.xpm )
            self.ATMega.setText("ATMega disconnected")
        elif status==STATUS.BUSY:
            wpm = QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/wait.png").scaled(17,17)
            self.ATMegaStatus.setPixmap( self.wpm )
            self.ATMega.setText("ATMega busy")
        self.ATMegaStatus.show()

    #set Joystick status (change image)
    def joystickEnabled(self, status):
        if status==STATUS.ENABLED:
            vpm = QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/V.png").scaled(17,17)
            self.joystickStatus.setPixmap( self.vpm )
            self.joystick.setText("Joystick connected")
        elif status==STATUS.DISABLED:
            xpm = QtGui.QPixmap(PATH_ROS+"/politocean/scripts/gui/X.png").scaled(17,17)
            self.joystickStatus.setPixmap( self.xpm )
            self.joystick.setText("Joystick disconnected")

    #update sensors widget
    def update_sensors(self, depth, pitch, roll):
        self.depth.display(float(str("{0:.3f}".format(depth))))
        self.pitch.setText(("{0:.2f}".format(pitch)+"°").decode("utf-8"))
        self.roll.setText(("{0:.2f}".format(roll)+"°").decode("utf-8"))

    #get commands from console
    def getCommands(self):
        comm = self.command
        self.command = ""
        return comm

    #get running state
    def isRunning(self):
        return self.running
