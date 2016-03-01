#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters
import cv2

# Ros libraries
import roslib
import rospy
import zlib
import socket
import time
import ach
import sys
import time
from ctypes import *
# Ros Messages
from sensor_msgs.msg import Image
from bountybondsman.msg import task
from bountybondsman.msg import success

VERBOSE=False
HEIGHT = 240
WIDTH = 320
CHANNELS = 3
INPORT = 8052
OUTPORT = 15000


# Ach files
class cloud(Structure):
    _pack_ = 1
    _fields_ = [("image"  , c_ubyte*320*240*3)]


class image_feature:

    def __init__(self):
        '''Initialize ros subscriber'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # this list should be in ros...
        self.dataCenters = [('10.112.120.247', INPORT), ('104.131.172.175', INPORT), ('159.203.67.159', INPORT), ('45.55.143.53', INPORT), ('45.55.143.47', INPORT), ('159.203.47.107', INPORT), ( '159.203.47.108',INPORT), ('159.203.47.109', INPORT), ('159.203.47.110', INPORT), ('129.174.121.166', INPORT)]
        self.startTime = -1
        self.nextTime = -1
        self.id = 1
        self.initBounty = 30
        # publish a task message
        # includes type/name (image blob) initial bounty, round trip deadline
        # publish reward message
        # winner ip, total time, reward
        self.THRESHOLD = 10000
        self.taskPub = rospy.Publisher('/bountybondsman/task', task, queue_size=10)

        self.subscriber = rospy.Subscriber("/bountybondsman/success",
            success, self.successCallback,  queue_size = 1)


        self.s = ach.Channel('image_chan')
        self.state = cloud()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/image_raw",
            Image, self.callback,  queue_size = 1)
        print 'Finished creating the image_features'
        ## need to then subscribe to
        if VERBOSE :
            print "subscribed to /camera/image/compressed"

    def successCallback(self, ros_data):
        ''' using the success message reorder the list of bounty hunters'''
        if ros_data.taskID == -1:
            ## then I'm done!
            rospy.signal_shutdown("received succ with -1 id")


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
           
        self.image = bytearray(ros_data.data)
      
        self.state.image = self.image
        self.s.put(state)
        
    def publishTask(self):
        ''' task message is published
            string taskName
            string[] bountyHunters
            float64 initialBounty
            float64 bountyRate
            float64 deadline
            uint32 inputPort
            uint32 outputPort
        '''
        msg = task()
        msg.taskName = "visualServoing"
        # me, nyc, sfo
        msg.bountyHunters = ["10.112.120.247", "104.131.172.175", "159.203.67.159", "45.55.143.53", "45.55.143.47", "159.203.47.107", "159.203.47.108", "159.203.47.109", "159.203.47.110", "129.174.121.166"]
        msg.initialBounty = self.initBounty
        msg.bountyRate = 1
        msg.deadline = 30
        msg.inputPort = INPORT
        msg.outputPort = OUTPORT
        print 'publishing the task'
        self.taskPub.publish(msg)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('bountycamera', anonymous=True)
    ic.publishTask()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)

