#!/usr/bin/env python
## ACH and comm stuff
import ach
import subprocess
from ctypes import *
import zlib


## ros stuff
import rospy
import time
import signal, os
import sys

## camera stuff
import numpy as np
import cv2

## messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
#from DataCollector import DataCollector
HEIGHT = 240
WIDTH = 320
CHANNELS = 3




class TaskData(Structure):
    _fields_ = [('id', c_double),
                ('img', c_char_p)]

class VelDat(Structure):
    _fields_ = [('forwardVelocity', c_double),
                ('angularVelocity', c_double),
                ('id', c_double)]


class BountyCloudVS:

    def __init__(self):


        f = open('ipaddresses.txt', 'r')
        self.servers = f.readlines()
        f.close()


        ## build list of channels for sending and receiving
        self.taskSendChannels = []
        self.taskRecvChannels = []
        for server in self.servers:

            imgTaskChanName = server.replace(".", "").replace("\n","") + "VSTaskImg"
            self.taskSendChannels.append(ach.Channel(imgTaskChanName)) # sending on
            respChan = server.replace(".", "").replace("\n", "") + "VSResp"
            self.taskRecvChannels.append(ach.Channel(respChan)) # receiving from

        print("done setting up now testing latency...")



        for testID in range(0, len(self.servers)):
            self.latency = []
            for i in range(0,10000):
                recvDat = VelDat()
                self.beginSend = time.time()
                self.taskSendChannels[testID].put('reducedTask')
                self.taskRecvChannels[testID].get(recvDat, wait=True, last=True)
                self.recvDatTime = time.time()

                self.latency.append(self.recvDatTime - self.beginSend)
                print("latency for {} count is {} is {}".format(self.servers[testID], i, self.recvDatTime - self.beginSend))


            f = open("latency"+self.servers[testID].replace(".", "").replace("\n",""), "w")
            f.write("\n".join(str(x) for x in self.latency))
            f.close()
            print("finished latency test and have written out to "+"latency"+self.servers[testID])

def main(args):
    '''Initializes and cleanup ros node'''
    ic = BountyCloudVS()
    rospy.init_node('cloudbot_vs', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)
