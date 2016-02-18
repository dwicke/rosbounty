#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np

# Ros libraries
import roslib
import rospy

import socket

# Ros Messages

VERBOSE=False

class bondsman:

    def __init__(self):
        '''Initialize ros subscriber'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dataCenters = [('10.112.120.196', 8052)]

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/image_raw",
            Image, self.callback,  queue_size = 1)

        if VERBOSE :
            print "subscribed to /camera/image/compressed"


    def successCallback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''


    def taskCallback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''




def main(args):
    '''Initializes and cleanup ros node'''
    myBondsman = bondsman()
    rospy.init_node('bondsman', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)

