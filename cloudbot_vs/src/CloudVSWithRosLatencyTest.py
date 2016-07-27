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


        self.testLatency = True
        self.latency = []
        # how long do we wait for a message from the servers
        self.waitTime = 0.01 ## 100 hz

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

        print("done setting up now just waiting to get an image...")
        self.id = 0.0
        self.failCount = 0
        self.succCount = 0


        self.pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber("/camera/image_raw", Image, self.callback,  queue_size = 1)
        rospy.on_shutdown(self.shutdown)



    def shutdown(self):
        # stop the robot
        self.robot_vel(0,0)
        # write out the success and fails?

    def robot_vel(self, forward, angular):
        twist = Twist()
        twist.linear.x = forward
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angular
        self.pub.publish(twist)

    ## t0 -----callback-----
    def callback(self, ros_data):
        print("got an image!!!")
        self.curTime = time.time()
        self.timeDelta = self.curTime - self.prevTime
        self.prevTime = self.curTime
        print("time from last image is {}".format(self.timeDelta))
        ### get image data from camera and process it (don't use ROS just use openCV)
        self.image = bytearray(ros_data.data)

        self.image = np.array(self.image, dtype="uint8").reshape(HEIGHT,WIDTH,CHANNELS)

        hsv = cv2.cvtColor(self.image,cv2.COLOR_BGR2HSV)
        ORANGE_MIN = np.array([5, 50, 50],np.uint8)
        ORANGE_MAX = np.array([15, 255, 255],np.uint8)
        reducedimg = cv2.inRange(hsv,ORANGE_MIN, ORANGE_MAX)

        taskReq = str(self.id) + "," + reducedimg.tostring()
        reducedTask = zlib.compress(taskReq, 3)

        self.id = self.id + 1.0
        print("sending image to the hunters")
        ### send image to bounty hunters (so will need a seperate channel to send images)

        self.beginSend = time.time()
        for sendChan in self.taskSendChannels:
            sendChan.put(reducedTask)

        winner = None

        while winner == None:
            for recvChan in self.taskRecvChannels:
                recvDat = VelDat()
                recvChan.get(recvDat, wait=True, last=True)
                self.recvDatTime = time.time()
                winner = recvDat


        if self.testLatency == True:
            self.latency.append(self.recvDatTime - self.beginSend)
            print("latency for {} is {}".format(self.id, self.recvDatTime - self.beginSend))
            if self.id == 10000:
                ## write the list to a file
                f = open("latency"+self.servers[0], "w")
                f.write("\n".join(str(x) for x in self.latency))
                f.close()
                print("finished latency test and have written out to "+"latency"+self.servers[0])

        if winner == None:
            ### if it times out restart the loop and count as a fail
            self.failCount += 1
            print("did not get a response")
        else:
            ### if we have a msg count as success and then send commands to the servos
            self.succCount += 1
            self.robot_vel(winner.forwardVelocity, winner.angularVelocity)
            print("Got a resonse and set the robot velocity {} {}".format(winner.forwardVelocity, winner.angularVelocity))

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
