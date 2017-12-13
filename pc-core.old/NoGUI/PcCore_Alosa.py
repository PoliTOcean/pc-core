import pygame
import serial
from pygame import locals
from time import sleep
import numpy as np

arduino=serial.Serial("/dev/ttyUSB0",9600)
#arduino=serial.Serial("/dev/cu.usbserial-A900XW2W",9600)
#arduino=serial.Serial("/dev/ttyACM0",9600)

pygame.init()
pygame.joystick.init() # main joystick device system

button=np.zeros(16)

try:
    j = pygame.joystick.Joystick(0) # istanza del joystick
    j.init()
    print ('Enabled joystick: ' + j.get_name())

except pygame.error:
    print ('no joystick found.')

while 1:

    for i in range(0,4):
        button[-1-i]=round(j.get_axis(i), 2)
        
    for i in range(0,12):
        button[i]=j.get_button(i)

    comm=""
    for i in range(0,len(button)):
        comm+=str(button[i]) + ' '
    comm+='\n'
    print(comm)
    arduino.write(comm.encode())

    sleep(0.1)

    pygame.event.clear()
