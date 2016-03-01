#!/usr/bin/env python
import numpy as np
import cv2
import zlib
import socket
import time
import ach
import sys
import time
from ctypes import *


def decideWinner(recvData):
    maxID = -1
    curWinner = None
    winnerIP = None
    hzRecv = False
    if recvData == None:
        return curWinner, winnerIP
    for datum in recvData:
        if datum[0] != 'connected':
            data_ar = datum[0].split(',')
            recvID = int(data_ar[2])
            if recvID > maxID:
                maxID = recvID
                curWinner = data_ar
                winnerIP = datum[1][0]

    return curWinner, winnerIP

def robot_vel(forward, angular):
    twist = Twist()
    twist.linear.x = forward
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angular
    pub.publish(twist)



INPORT = 8052

# set up the connection to the servers
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dataCenters = [('10.112.120.247', INPORT), ('104.131.172.175', INPORT), ('159.203.67.159', INPORT), ('45.55.143.53', INPORT), ('45.55.143.47', INPORT), ('159.203.47.107', INPORT), ( '159.203.47.108',INPORT), ('159.203.47.109', INPORT), ('159.203.47.110', INPORT), ('129.174.121.166', INPORT)]



# tell the servers to send stuff here
udpCon = ConnectionManager('udp')
udpCon.addClient('10.112.120.247', port)
# NY
udpCon.addClient('104.131.172.175', port)
udpCon.addClient('159.203.67.159', port)
udpCon.addClient('45.55.143.47', port)
udpCon.addClient('45.55.143.53', port)
# TOR
udpCon.addClient('159.203.47.110', port)
udpCon.addClient('159.203.47.109', port)
udpCon.addClient('159.203.47.108', port)
udpCon.addClient('159.203.47.107', port)

udpCon.addClient('129.174.121.166', port)


udpCon.send('HI I am udp motion message')
while len(udpCon.recv(1)) == 0:
    udpCon.send('HI I am udp motion message')
print 'Recv response'


# Ach files
class cloud(Structure):
    _pack_ = 1
    _fields_ = [("image"  , c_ubyte*320*240*3)]


s = ach.Channel('image_chan')
state = cloud()

id = 1
preID = 0
curFor = -5.0
curAng = -5.0

totalInc = 0.0
succInc = 0.0

while True:
   tick = time.time()
   [statuss, framesizes] = s.get(state, wait=False, last=True)
   

   # process the image
   image = np.array(state.image, dtype="uint8").reshape(HEIGHT,WIDTH,CHANNELS)
   hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
   ORANGE_MIN = np.array([5, 50, 50],np.uint8)
   ORANGE_MAX = np.array([15, 255, 255],np.uint8)
   reducedimg = cv2.inRange(hsv,ORANGE_MIN, ORANGE_MAX)
   data = "%s,%s,%s" % (str(id), str(tick), reducedimg.tostring())

   for datacenter in dataCenters:
            self.sock.sendto(zlib.compress(data, 3), datacenter)
   t = time.time()
    
   recvData = udpCon.recv(T-(t-tick))
   data_ar, addr = decideWinner(recvData)
   if addr != None:
        recvID = int(data_ar[2])
        recvTS = float(data_ar[3])
        taskName = data_ar[4].strip()
        forward = float(data_ar[0])
        ang = float(data_ar[1])
        if recvID > preID:
            preID = recvID
            if curFor != forward or curAng != ang:
                robot_vel(forward, ang)
                curFor = forward
                curAng = ang	

        tock = time.time()
        if T - (tock-tick) > 0:
            time.sleep(T - (tock - tick))




