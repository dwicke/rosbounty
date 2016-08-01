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
from ConnectionManager import ConnectionManager
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

        self.tcpCon = ConnectionManager('tcp')
        self.tcpCon.addClient('169.254.99.212', 9003)
        #self.tcpCon.addClient('192.168.2.5', 9002)


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
        self.tcpCon.send(str(reducedimg))



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
