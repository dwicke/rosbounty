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

# Ros Messages
from sensor_msgs.msg import Image
from bountybondsman.msg import task

VERBOSE=False
HEIGHT = 240
WIDTH = 320
CHANNELS = 3

class image_feature:

    def __init__(self):
        '''Initialize ros subscriber'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # this list should be in ros...
        self.dataCenters = [('10.112.120.196', 8052)]

        # publish a task message
        # includes type/name (image blob) initial bounty, round trip deadline
        # publish reward message
        # winner ip, total time, reward



        #self.sock.connect(('10.112.120.213', 8052))
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/image_raw",
            Image, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"



    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
       	if VERBOSE :
		print 'received image of size: "%d" x "%d"' % (ros_data.width, ros_data.height)
    	if VERBOSE :
    		print ' len of data = "%d"' %  (len(ros_data.data))
        self.image = bytearray(ros_data.data)
        image = np.array(self.image, dtype="uint8").reshape(HEIGHT,WIDTH,CHANNELS)
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        ORANGE_MIN = np.array([5, 50, 50],np.uint8)
        ORANGE_MAX = np.array([15, 255, 255],np.uint8)
        reducedimg = cv2.inRange(hsv,ORANGE_MIN, ORANGE_MAX)
        #print len(zlib.compress(reducedimg.tostring(), 9))
    	self.sock.sendto(zlib.compress(reducedimg.tostring(), 9), self.dataCenters[0])



def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)

