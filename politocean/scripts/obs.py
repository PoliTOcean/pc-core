#!/usr/bin/env python
''' obs node'''
import serial
import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Vector3
import time

def Trova(Stringa, Carattere, Indice):
    while Indice < len(Stringa):
        if Stringa[Indice] == Carattere:
            return Indice
        Indice = Indice + 1
    return -1

def obs():
    pub = rospy.Publisher('obs_pub', Vector3, queue_size=0)
    rospy.init_node('obs', anonymous=False)
        
    s = serial.Serial('/dev/ttyUSB0', 9600, timeout=10)
    stri = s.readline()
    
    command = Vector3()
    
    command.x = 0
    command.y = 0
    command.z = 0
        
    while not rospy.is_shutdown():
        stri = ''
        while(len(stri)==0):
            stri = s.readline()
            time.sleep(0.01)
            
        print(stri)
        
        if stri[0]=='<' and len(stri)>40:
            ind = Trova(stri, '=', 0)
            ind2 = Trova(stri, ' ', ind)
            V = stri[ind+1:ind2-1]
            command.z = float(V)
            
            if len(stri)>55:
                ind = Trova(stri, '=', ind+1)
                ind2 = Trova(stri, ' ', ind2+1)
                X = stri[ind+1:ind2-1]
                
                ind = Trova(stri, '=', ind+1)
                ind2 = Trova(stri, ' ', ind2+1)
                Y = stri[ind+1:ind2-1]
                
                command.x = float(X)
                command.y = float(Y)
            
            if len(stri)>90:
                ind = Trova(stri, ':', 0)
                data = stri[ind+1:-1]
                data = np.fromstring(data, dtype=float, sep=',')
                plt.plot(data)
                plt.show()
                break
            
            print command
            pub.publish(command)

if __name__ == '__main__':
    try:
        obs()
    except rospy.ROSInterruptException:
        pass
