#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from PyQt4 import QtGui,QtCore

class Window(QtGui.QMainWindow):
	
	#inizializzo finestra principale
	def __init__(self):
		super(Window,self).__init__()
		self.setGeometry(50,50,500,300)
		self.setWindowTitle('PoliTOcean')
		self.createButton()
# moved initialization of the node in the __init__ (from the talker definition), when the node starts it has to create (or connect to) the topic
# changed name of the topic according to the ROS standard
		self.pub = rospy.Publisher('gui_chatter', String, queue_size=10)
    		rospy.init_node('guiTalker', anonymous=True)

	#creo bottone
	def createButton(self):
		btn = QtGui.QPushButton("Send to Topic",self)
		btn.clicked.connect(self.publish)
		#connetto clik con la funzione publish
		btn.resize(150,80)
		btn.move(250,150)
		self.show()

# changed name from talker to publish in order to make it more consistent to the corrected functionality
	#funzione che manda messaggi al topic gui_chatter
	def publish(self):
		print("Sto inviando un messaggio..")

		#verifico che ci sia un ROS master attivo
		if not rospy.is_shutdown():
			mex = "I'm a message."
			rospy.loginfo(mex)
			self.pub.publish(mex)
			print("Messaggio inviato")

		#il messaggio non viene pubblicato
		else:
			print("Impossibile inviare messaggio")

#funzione di inizializzazione
def run():
	app = QtGui.QApplication(sys.argv)
	main = Window()
	sys.exit(app.exec_())

#main program
run()
