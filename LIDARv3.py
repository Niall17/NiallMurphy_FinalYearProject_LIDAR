#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  3 16:25:00 2021

@author: Niall Murphy
"""

import serial    
import math
import numpy as np
import re
from collections import deque                                 #import deque and counter to calculate a moving mode 
from collections import Counter
import serial.tools.list_ports                                 #used to detect arduino port

ports = serial.tools.list_ports.comports()                      #add used com ports to a list


a=np.zeros((5,7),dtype=np.int)                                  #arrays for each LIDAR and a master array
b=np.zeros((5,7),dtype=np.int)
c=np.zeros((5,7),dtype=np.int)
mast=np.zeros((5,7),dtype=np.int)
zer=np.zeros((5,7),dtype=np.int)                                #empty array for clearing ldr scans

q=deque(maxlen=3)                                               #que with maxlen so old values are dropped off end as new values enter

Occ=0
mainOcc=0

dispMast=False                                                #boolean to display the master grid to user for debugging purposes


def clearCorners():                                             #function to clear edges of arrays where LIDAR's may have detected eachother      
    global a
    global b
    global c
    
    a[4][0]=0                                                  #clear any corner values where the LIDAR's are positioned
    a[4][6]=0
    
    
    
    b[0][0]=0
    b[4][6]=0
    
    
    c[0][0]=0
    c[4][0]=0
    


def updateMaster():                                             #function to update master array
    global a
    global b
    global c
    global mast
    global Occ
    global mainOcc
    mast=np.copy(zer)                                           #clear master on every update to wipe old reads
    clearCorners();
    
    for row in range(0,5):                                     #loop over row column pairs and commit similarities to master
        for col in range(0,7):
            if((a[row][col]==1)and((a[row][col]==c[row][col])or(a[row][col]==b[row][col]))):
                mast[row][col]=1
            if((b[row][col]==1)and(b[row][col]==c[row][col])):
                mast[row][col]=1
    
    
    if(Occ!=np.count_nonzero(mast)):
        Occ=np.count_nonzero(mast)
    
    q.append(Occ)                                         #append new occupancy calc to que
    mainOcc=Counter(q).most_common(1)[0][0]                           #save the value that occurs most often (mode) and use as occupancy
    print(mainOcc)
#    print(a)
#    print(b)
#    print(c)
    global dispMast
    if dispMast:
        print(mast)                                             #only display if user requests
    

    a=np.copy(zer)                                              #clear arrays after 90 degree scan
    b=np.copy(zer)   
    c=np.copy(zer) 
    
    


def interpret(line):                                            #function to interpret data from arduino
    line =str(line)                                              #Cast to string
    #print(line)
    values=re.findall(r"[-+]?\d*\.\d+|\d+",line)                #extract float numbers from line as string
    values=[float(i) for i in values]                           #convert list of string numbers to floats
    #print(values)                                             #print recieved values
    angledeg=values[3]                                             #fourth integer is angle of Stepper
    values=values[:-1]
    #############################
    #   More Work on Constants to add
    #############################
    if ((angledeg>=0 and angledeg<=15)or(angledeg>=75 and angledeg<=90)):   #add constant to values based on angle of sensor
        values=[x + 4.575 for x in values]
    elif ((angledeg>=16 and angledeg<=30)or(angledeg>=61 and angledeg<=74)):
        values=[x + 5.008 for x in values]
    else:
        values=[x + 5.441 for x in values]
        
    values=[x / 3.70455 for x in values]                        #scale to ft (1ft:37.0455mm)
    angle=math.radians(angledeg)                                   #convert to radians   
    ldrA=values[0]                                              #first integer is lidarA
    ldrB=values[1]                                              #second integer is lidarB
    ldrC=values[2]                                              #third integer is lidarC
    #print(ldrC)
    
    si=math.sin(angle)                                          #get sin and cos of angle
    co=math.cos(angle)
    
    ax=co*ldrA
    ay=si*ldrA
    bx=si*ldrB                                                  #for ease of use with logic (kept on relative scale of 0-(21or15))
    by=co*ldrB
    cx=co*ldrC
    cy=si*ldrC
    #print(cx)
    #print(cy)
    
    ldrAx=int(ax/3)                                               #calc x and y co-ords for each lidar
    ldrAy=int(ay/3)
    
    ldrBx=int(bx/3)
    ldrBy=int((15-(by))/3)                                       #cast to int to truncate 
    
    ldrCx=int((21-(cx))/3)
    ldrCy=int((15-(cy))/3)
    #print(ldrCx)
    #print(ldrCy)
    
                                                                #Conditions to update arrays
    if  ((ax<21)and(ay<15)) :      
        if ((ldrAx<7)and(ldrAy<5)):                        
            global a
            a[ldrAy][ldrAx]=1
    
    if ((bx<21)and(by<15)) : 
        if ((ldrBx<7)and(ldrBy<5)):
            global b
            b[ldrBy][ldrBx]=1
    
    if ((cx<21)and(cy<15)) : 
        if ((ldrCx<7)and(ldrCy<5)):
            global c
            c[ldrCy][ldrCx]=1
    
    if ((angledeg==0)or(angledeg==90)):                               #update after every 90 degree scan
        updateMaster()
    

if __name__=='__main__':
    for p in ports:
        if "ACM" in p.description:
            ard='/dev/'+p.description    
            print("Arduino Detected")
    
    
    
    ser=serial.Serial(ard,115200,timeout=1)          #Inititalise serial communication via serial port 
    ser.flush()                                                 #flush serial port 
    
    
    display=input("Would you like to dispay the Master Occupancy Grid along with the output?[y/n]: ")
    if 'y' in display:                                          #ask user if they would like to see the master occupancy grid on output
        
        dispMast=True
    
    go=input("Press 'y' and hit enter to begin in debug mode, enter 'n' for normal mode: ")
    
    if 'y' or 'n' in go: #User input to initiate process
        
        ser.write(go.encode())
        print("begin")
        
        while True:
            if ser.in_waiting>0:
                line=ser.readline().decode('utf-8').rstrip()
                print(line)
                line=str(line)
                if line=='start':
                    break
    
    
    while True:                                                 #loop forever
        if ser.in_waiting>0:
            line=ser.readline().decode('utf-8').rstrip()        #read in serial line
            #print(line)                                         #print
            if 'wait' in line:                                  #if in debug mode, wait for user to continue
                res=input("Currently in debug mode, please hit return key to continue: ")
                ser.write(b"y\n")
            else:
                interpret(line)                                     #interpet line
            
