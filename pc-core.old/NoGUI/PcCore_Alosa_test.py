import pygame
import serial
from pygame import locals
from time import sleep
import numpy as np
import datetime

arduino=serial.Serial("/dev/ttyUSB0",9600)
#arduino=serial.Serial("/dev/cu.usbserial-A900XW2W",9600)
#arduino=serial.Serial("/dev/ttyACM0",9600)

pygame.init()
pygame.joystick.init() # main joystick device system

button=np.zeros(16)
path = 'Temperatura.txt'
##file_temp=open(path,'w')
##file_temp.write('Minute, Second, Temperature\n')
##file_temp.close()

try:
    j = pygame.joystick.Joystick(0) # istanza del joystick
    j.init()
    print ('Enabled joystick: ' + j.get_name())

except pygame.error:
    print ('no joystick found.')

arduino.flush()

while 1:
    if arduino.in_waiting:
        temp=arduino.readline()
        time=datetime.datetime.now()
        file_temp=open(path,'a')
        file_temp.write(str(time.minute)+ ' '+str(time.second)+' '+str(time.microsecond)+' '+temp.decode("utf-8"))
        file_temp.close()
        print(str(time.minute)+ ' '+str(time.second)+' '+str(time.microsecond)+' '+temp.decode("utf-8"))
    for i in range(0,4):
        button[-1-i]=round(j.get_axis(i), 2)

    for i in range(0,12):
        button[i]=j.get_button(i)

    comm=""
    for i in range(0,len(button)):
        comm+=str(button[i]) + ' '
    comm+='\n'
    #print(comm)
    arduino.write(comm.encode())

    sleep(0.1)

    pygame.event.clear()
