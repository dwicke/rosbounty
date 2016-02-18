#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters


# Ros libraries
import roslib
import rospy
import zlib
import socket

# Ros Messages
from sensor_msgs.msg import Image

VERBOSE=False

class image_feature:

    def __init__(self):
        '''Initialize ros subscriber'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dataCenters = [('10.112.120.203', 8052)]
        #self.sock.connect(('10.112.120.213', 8052))
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/image_raw",
            Image, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"


    ## pixels are ordered BGR
    def inrange(self, lowrange, highrange):
        curColor = 0
        binaryImage = [1 for i in xrange(self.imageWidth * (self.imageHeight - 40)]
        curIndex = 0
        for val in self.image:
	    #print "'%d'" % (val)
	    if (curIndex >= self.imageWidth * 40) and (val < lowrange[curColor] or val > highrange[curColor]):
                binaryImage[curIndex] &= 0
            curColor = (curColor + 1) % 3
            if curColor == 0:
                curIndex += 1
        return binaryImage




    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
       	if VERBOSE :
		print 'received image of size: "%d" x "%d"' % (ros_data.width, ros_data.height)
    	if VERBOSE :
    		print ' len of data = "%d"' %  (len(ros_data.data))
        self.image = bytearray(ros_data.data)
        self.imageWidth = ros_data.width
        self.imageHeight = ros_data.height
        #print self.inrange((0,43, 215), (80,90,255))
	#compressedImage = zlib.compress(''.join(map(str, self.inrange((0,43, 215), (80,90,255)))), 2)
        #print 'Length of image i amd sending is "%d"' % (len(compressedImage))
        reducedimg = ''.join(map(str, self.inrange((0,43, 215), (80,90,255))))
        print len(reducedimg)
    	self.sock.sendto(reducedimg, self.dataCenters[0])



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

