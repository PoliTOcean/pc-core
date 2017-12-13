from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import timeit
from time import sleep

PATH = os.environ['ROS_PACKAGE_PATH']
PATH = PATH.split(':')
PATH_ROS = PATH[0]
form_class = uic.loadUiType(PATH_ROS+"/politocean/scripts/gui/simple.ui")[0]

def publishErrors(err):
    try:
        errors_pub.publish(err)
    except:
        print("!!! Closed errors topic !!!")

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


        #the btn was a button that start updating of frames taked from ROS node
        self.btn.clicked.connect(self.start_clicked)

        #boolean for start the connection
        self.running = False

        #cam indexes
        self.cam = [0, 1, 2]

        #commands (it will be needed for the console)
        self.command = ""

        #set a timer to call setFrame function
        timer = QTimer(self)
        self.connect(timer, SIGNAL("timeout()"), self.setFrame)
        timer.start(40)

        self.frm = [None, None, None]



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

    #toggle running state
    def start_clicked(self):
        if not self.running:
			#start of connection
            self.running = True;
            self.btn.setText('Starting...')
        else:
        	#end of connection
            self.btn.setText('Click to Start')
            self.running = False;

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
