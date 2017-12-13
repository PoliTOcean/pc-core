#!/usr/bin/env python

from PyQt4 import QtCore, QtGui, uic
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
import sys
import rospy
import cv2
import os

PATH = os.environ['ROS_PACKAGE_PATH']
PATH = PATH.split(':')
PATH_ROS = PATH[0]
form_class = uic.loadUiType(PATH_ROS+"/politocean/scripts/simple.ui")[0]

class ImageWidget(QtGui.QWidget):
	
	def __init__(self,parent=None):
		super(ImageWidget, self).__init__(parent)
		self.image = None		
		self.running = False
	
	
	def setSize(self,width,height):
		self.window_width  = width
		self.window_height = height
		#Questo carica l'immagine di "segnale assente" 	
		#self.timeout = cv2.imread(PATH_ROS+"/politocean/scripts/timeout.png")
		#self.tim = self.resize(self.timeout)	

	def setImage(self, image):
		self.img = image
		self.image = self.resize(image)
		sz = self.image.size()
		self.setMinimumSize(sz)
		#the update fuction call the paintEvent
		if self.running:
			self.update()
		

	def paintEvent(self, event):
		qp = QtGui.QPainter()
		qp.begin(self)
		if self.image and self.running:
		    qp.drawImage(QtCore.QPoint(0, 0), self.image)		    
		'''
		else:
			qp.drawImage(QtCore.QPoint(0, 0), self.tim)
		'''
		qp.end()
		
	   
	def setRunning(self,boolean):
		self.running = boolean;
		self.update()
	

	def resize(self,img):
		img_height, img_width, img_colors = img.shape
		self.scale_w = float(self.window_width) / float(img_width)
		self.scale_h = float(self.window_height) / float(img_height)
		self.scale = min([self.scale_w, self.scale_h])
		img = cv2.resize(img, None, fx=self.scale_w, fy=self.scale, interpolation = cv2.INTER_CUBIC)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		self.height, self.width, self.bpc = img.shape
		self.bpl = self.bpc * self.width
		return QtGui.QImage(img.data, self.width, self.height, self.bpl, QtGui.QImage.Format_RGB888)
		
	


class Window(QtGui.QMainWindow,form_class):
	
	#init of Main Window
	def __init__(self,parent=None):
		QtGui.QMainWindow.__init__(self, parent)		
		self.setupUi(self)
		self.setWindowTitle('PolitOcean')
		self.case = False
		
		#Da inserire in una classe CheckBox
		self.check1 = QtGui.QCheckBox('cam1',self)
		self.check1.move(0,325)
		self.check1.setChecked(True)

		self.check2 = QtGui.QCheckBox('cam2',self)
		self.check2.move(0,350)
		
		self.check3 = QtGui.QCheckBox('cam3',self)
		self.check3.move(0,375)
		
		self.check1.stateChanged.connect(lambda:self.goCheck(self.check1,self.check2,self.check3))
		self.check2.stateChanged.connect(lambda:self.goCheck(self.check2,self.check1,self.check3))
		self.check3.stateChanged.connect(lambda:self.goCheck(self.check3,self.check1,self.check2))
				
		self.cam = 1
		#fino a qui 
				
		self.btn.clicked.connect(self.start_clicked)
		
		#Dimension for resize cam frames.
		self.size  = self.mainCam.frameSize()
		self.size2 = self.cam2.frameSize()
		self.size3 = self.cam3.frameSize()				
				
		#init custom Widget that print Image on PyQT
		self.mainCam = ImageWidget(self.mainCam)
		self.mainCam.setSize(self.size.width(),self.size.height())
		
		self.cam2 = ImageWidget(self.cam2)
		self.cam2.setSize(self.size2.width(),self.size2.height())
		
		self.cam3 = ImageWidget(self.cam3)
		self.cam3.setSize(self.size3.width(),self.size3.height())
	
		#boolean for start the connection	
		running = False		

 	def start_clicked(self):
		if not self.case:
			self.mainCam.setRunning(True)
			self.cam2.setRunning(True)
			self.cam3.setRunning(True)
			self.btn.setText('Running...')
			self.case = True
		else:
			self.mainCam.setRunning(False)
			self.cam2.setRunning(False)
			self.cam3.setRunning(False)
			self.btn.setText('Click to Start')
			self.case = False
				    	    

	def closeEvent(self, event):
		global running
		running = False

	def getCam(self,num):
		if(num == 1):
			return self.mainCam;
		if(num == 2):
			return self.cam2;
		if(num == 3):
			return self.cam3;
	
	def getLabel(self):
		return self.label
		
	#Da inserire nella classe dei CheckBox
	def goCheck(self,sel,o1,o2):
		
		if sel.text() == "cam1":
			if sel.isChecked():	
				self.cam = 1
				o1.setChecked(False)
				o2.setChecked(False)
		if sel.text() == "cam2":
			if sel.isChecked():	
				self.cam = 2
				o1.setChecked(False)
				o2.setChecked(False)
		if sel.text() == "cam3":
			if sel.isChecked():	
				self.cam = 3
				o1.setChecked(False)
				o2.setChecked(False) 	
		
	def getState(self):
		return self.case
	#Fino a qui					
				
	def getCamState(self):
		return self.cam
		
	


class Converter:	
	def __init__(self,window):
		self.window = window
		#Cv Bridge converts the ROS image to OpenCv Image
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/webcam/image_raw",Image,self.callback)

	def callback(self,data):				
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
			#DEBUG PRINT
			#print("I'm converting")
		except CvBridgeError as e:
				print(e)
				
		#selezione del CheckBox cam1
		if self.window.getCamState() == 1:							
			self.window.getCam(1).setImage(img)
			if self.window.getState():
				self.window.getCam(1).setRunning(True)
				self.window.getCam(2).setRunning(False)
				self.window.getCam(3).setRunning(False)
			
		#selezione del CheckBox cam2
		if self.window.getCamState() == 2:
			self.window.getCam(2).setImage(img)
			if self.window.getState():
				self.window.getCam(2).setRunning(True)
				self.window.getCam(1).setRunning(False)
				self.window.getCam(3).setRunning(False)
		
		#selezione del CheckBox cam3
		if self.window.getCamState() == 3:
			self.window.getCam(3).setImage(img)
			if self.window.getState():
				self.window.getCam(3).setRunning(True)
				self.window.getCam(1).setRunning(False)
				self.window.getCam(2).setRunning(False)

		#DEBUG IMAGE -> i saw if the opencv standard frame reads the image from ROS
		#cv2.imshow("Image window", img)	
		#cv2.waitKey(3)
	

''' PROVA PER UN LISTENER DI TOPIC TEMPERATURA
class TimeListener:
	def __init__(self,window):
		self.window = window		
		self.temp = rospy.Subscriber("tempTopic",Int32,self.callback)
	
	def callback(self,data):
		temp = str(data)
		vett = temp.split(": ")
		self.window.getLabel().setText(str("Temperatura: "+vett[1]+" gradi"))
'''

def main():	
	app = QtGui.QApplication(sys.argv)
	Wmain = Window(None)	
	Wmain.show()
	con = Converter(Wmain)
	#timeList = TimeListener(Wmain)
	rospy.init_node('gui_reading_cam',anonymous=True)
	sys.exit(app.exec_())	

main()
