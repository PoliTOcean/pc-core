import sys
import rospy
from std_msgs.msg import String
from PyQt4 import QtGui,QtCore


class Window(QtGui.QMainWindow):
	
	#inizializzo finestra principale
	def __init__(self):
		super(Window,self).__init__()
		self.setGeometry(50,50,500,300)
		self.setWindowTitle('PolitOcean')
		self.createButton()

	#creo bottone
	def createButton(self):
		btn = QtGui.QPushButton("Send to Topic",self)
		btn.clicked.connect(self.talker)
		 #connetto clik con fuction talker
		btn.resize(150,80)
		btn.move(250,150)
		self.show()

	#funzione che manda messaggi al topic GUIchatter
	def talker(self):
		pub = rospy.Publisher('GUIchatter', String, queue_size=10)
    		rospy.init_node('guiTalker', anonymous=True)
		print("Sto inviando un messaggio..")

		#verifico che ci sia un ROS master attivo
		if not rospy.is_shutdown():
			mex = "I'm a message."
			rospy.loginfo(mex)
			pub.publish(mex)
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
