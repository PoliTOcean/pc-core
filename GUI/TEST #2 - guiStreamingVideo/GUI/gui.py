from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import pyqtSignal, QSize, Qt
from PyQt4.QtGui import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import Queue
import roslib
import sys
import rospy
import cv2

form_class = uic.loadUiType("simple.ui")[0]
q = Queue.Queue()

class ImageWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        super(ImageWidget, self).__init__(parent)
        self.image = None

    def setImage(self, image):
        self.image = image
        sz = image.size()
        self.setMinimumSize(sz)

	#the update fuction call the paintEvent
        self.update()

    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        if self.image:
            qp.drawImage(QtCore.QPoint(0, 0), self.image)
        qp.end()


class Window(QtGui.QMainWindow,form_class):

    #init of Main Window
    def __init__(self,parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.setWindowTitle('PolitOcean')

        self.segn_ass = cv2.imread('segnale_assente.png')

        #image dimension -> "segnale_assente.png"
        height, width, bpc = self.segn_ass.shape

        #converter of cv2Image -> QImage
        self.QTsegn_ass = QtGui.QImage(self.segn_ass.data, width, height, bpc*width, QtGui.QImage.Format_RGB888)

        #politocean was created with setupUi fuction that read from simple.ui file
        self.politocean.resize(400,150)
        self.politocean.setPixmap(QtGui.QPixmap("politocean3.png"))
    	self.politocean.show()

        #the btn was a button that start updating of frames taked from ROS node
        self.btn.clicked.connect(self.start_clicked)

        #Dimension for resize cam frames.
        self.window_width = self.mainCam.frameSize().width()
        self.window_height = self.mainCam.frameSize().height()
        self.window_width2 = self.cam2.frameSize().width()
        self.window_height2 = self.cam2.frameSize().height()

        #init custom Widget that print Image on PyQT
        self.mainCam = ImageWidget(self.mainCam)
        self.cam2 = ImageWidget(self.cam2)
        self.cam3 = ImageWidget(self.cam3)

        #boolean for start the connection
        self.running = False

        #timer thread that callback the real drawing of all frames.
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)
    	self.timer.start(2)


    def start_clicked(self):
        if not self.running:
            #start of connection
            self.running = True;
            self.btn.setText('Starting...')
        else:
            #end of connection
            self.running = False;
            self.btn.setText('Click to Start')
            self.mainCam.setImage(self.QTsegn_ass)
            self.cam2.setImage(self.QTsegn_ass)
            self.cam3.setImage(self.QTsegn_ass)



    def update_frame(self):
        #this fuction is the core drawing system: i read an image from my queue that contains my cvImage frames
        #(next part was in italian sorry, i can't in Eglish xD)
            #Mi calcolo le proporzioni dell immagine e le adatto alla dimensione del Widget in cui devo metterla
        #una volta convertita l'immagine da cv a Qt la setto nel widget creato in precedenza
                #che si occupa di disegnarla a schermo

        if self.running:
            while not q.empty():
                self.btn.setText('Stop Cam')
                frame = q.get()
                img = frame["img"]

                img_height, img_width, img_colors = img.shape
                scale_w = float(self.window_width) / float(img_width)
                scale_h = float(self.window_height) / float(img_height)

                #two scales: 1 for the main cam, another for the secondary cams.
                scale_w2 = float(self.window_width2) / float(img_width)
                scale_h2 = float(self.window_height2) / float(img_height)
                scale = min([scale_w, scale_h])
                scale2 = min([scale_w2, scale_h2])

                #if scale == 0:
                #scale = 1
                #if scale2 == 0:
                #scale2 = 1

                img_tmp = img

                img = cv2.resize(img, None, fx=scale_w, fy=scale, interpolation = cv2.INTER_CUBIC)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                height, width, bpc = img.shape
                bpl = bpc * width
                image = QtGui.QImage(img.data, width, height, bpl, QtGui.QImage.Format_RGB888)

                img2 = cv2.resize(img_tmp, None, fx=scale_w2, fy=scale2, interpolation = cv2.INTER_CUBIC)
                img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
                height2, width2, bpc2 = img2.shape
                bpl2 = bpc2 * width2
                image2 = QtGui.QImage(img2.data, width2, height2, bpl2, QtGui.QImage.Format_RGB888)

                self.mainCam.setImage(image)
                self.cam2.setImage(image2)
                self.cam3.setImage(image2)


    def closeEvent(self, event):
        global running
        running = False


class Converter:
    def __init__(self):
        #Cv Bridge converts the ROS image to OpenCv Image
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/webcam/image_raw",Image,self.callback)

    def callback(self,data):
        frame = {}

        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame["img"] = img

        #DEBUG PRINT
        #print("I'm converting")
        except CvBridgeError as e:
            print(e)

        q.put(frame)


        #DEBUG IMAGE -> i saw if the opencv standard frame reads the image from ROS
        #cv2.imshow("Image window", img)
        #cv2.waitKey(3)


def main():

    app = QtGui.QApplication(sys.argv)
    main = Window(None)
    main.show()
    con = Converter()
    rospy.init_node('gui_reading_cam',anonymous=True)
    #rospy.spin()
    sys.exit(app.exec_())



main()

