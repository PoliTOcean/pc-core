'''
This script is imported from almost every other node, either GUI or ROV side.
It provides the interface to communicate over errors and messages topics.
'''
import roslib
import rospy
from std_msgs.msg import String
from time import sleep

#publishers
#errors
errors_pub = rospy.Publisher('errors', String, queue_size=9)
#messages
messages_pub = rospy.Publisher('messages', String, queue_size=5)

#publish errors over the topic
def publishErrors(nodeName, err):
    try:
        errors_pub.publish(nodeName+" ::: "+err)
    except:
        print("!!! Closed errors topic !!!")

#publish messages over topic
def publishMessages(nodeName, data):
    try:
        messages_pub.publish(nodeName+' ::: '+str(data))
    except rospy.ROSInterruptException as e:
        publishErrors(nodeName, "Messages publishing error: "+str(e))

def errMessInit():
    #send ??? to "active" topics
    for i in range(1,10):
        publishErrors("", "???")
        publishMessages("", "???")
        sleep(0.2)
