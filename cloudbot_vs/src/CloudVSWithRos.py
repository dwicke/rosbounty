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


        # how long do we wait for a message from the servers
        self.waitTime = 0.1 ## 25 hz

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
        self.id = 1.0
        self.failCount = 0
        self.succCount = 0
        self.prevTime = time.time()
        self.latency = []

        self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
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
        #self.curTime = time.time()
        #self.timeDelta = self.curTime - self.prevTime
        #self.prevTime = self.curTime
        #print("time from last image is {}".format(self.timeDelta))
        ### get image data from camera and process it (don't use ROS just use openCV)
        self.image = bytearray(ros_data.data)

        self.image = np.array(self.image, dtype="uint8").reshape(HEIGHT,WIDTH,CHANNELS)

        hsv = cv2.cvtColor(self.image,cv2.COLOR_BGR2HSV)
        ORANGE_MIN = np.array([5, 125, 100],np.uint8)
        ORANGE_MAX = np.array([20, 255, 255],np.uint8)
        reducedimg = cv2.inRange(hsv,ORANGE_MIN, ORANGE_MAX)

        taskReq = str(self.id) + "," + reducedimg.tostring()
        reducedTask = zlib.compress(taskReq, 3)

        self.id = self.id + 1.0
        #print("sending image to the hunters")
        ### send image to bounty hunters (so will need a seperate channel to send images)

        winner = None
        self.endRecvTime = 0.0

        self.beginSend = time.time()
        tock = self.beginSend + self.waitTime
        for sendChan in self.taskSendChannels:
            sendChan.put(reducedTask)

        recvDat = VelDat()
        while (time.time() < tock) and winner == None:
            for recvChan in self.taskRecvChannels:
                recvChan.get(recvDat, wait=False, last=True)
                if recvDat.id == (self.id - 1.0):
                    self.endRecvTime = time.time()
                    winner = recvDat
        if self.endRecvTime == 0.0:
            self.endRecvTime = time.time()
            print("didn't recv anything")

        print("latency = {} ".format(self.endRecvTime - self.beginSend))
        self.latency.append(self.endRecvTime - self.beginSend)


        if (tock - time.time()) > 0.001:
            time.sleep(tock-time.time())

        if winner == None:
            ### if it times out restart the loop and count as a fail
            self.failCount += 1
            print("did not get a response")
        else:
            ### if we have a msg count as success and then send commands to the servos
            self.succCount += 1
            self.robot_vel(winner.forwardVelocity, winner.angularVelocity)
            print("Got a resonse and set the robot velocity {} {}".format(winner.forwardVelocity, winner.angularVelocity))

        if (len(self.latency) == 500):
            f = open("latencybountyhunting", "w")
            f.write("\n".join(str(x) for x in self.latency))
            f.close()
            # f = open("succbountyhunting", "w")
            # f.write(str(self.succCount) + "," + str(self.failCount))
            # f.close()
            print("wrote out latency")


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
