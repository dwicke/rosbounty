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
from bountybondsman.msg import success

VERBOSE=False
HEIGHT = 240
WIDTH = 320
CHANNELS = 3
INPORT = 8052
OUTPORT = 15000

class image_feature:

    def __init__(self):
        '''Initialize ros subscriber'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # this list should be in ros...
        self.dataCenters = [('10.112.120.247', INPORT), ('104.131.172.175', INPORT),('45.55.11.33', INPORT)]
        self.id = 0
        self.initBounty = 30
        self.baseBounty = 30 # does not change
        # publish a task message
        # includes type/name (image blob) initial bounty, round trip deadline
        # publish reward message
        # winner ip, total time, reward
        self.THRESHOLD = 100
        self.taskPub = rospy.Publisher('/bountybondsman/task', task, queue_size=10)

        self.subscriber = rospy.Subscriber("/bountybondsman/success",
            success, self.successCallback,  queue_size = 1)

        #self.sock.connect(('10.112.120.213', 8052))
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/image_raw",
            Image, self.callback,  queue_size = 1)

        ## need to then subscribe to
        if VERBOSE :
            print "subscribed to /camera/image/compressed"

    def successCallback(self, ros_data):
        ''' using the success message reorder the list of bounty hunters'''
        self.lastSuccess = time.time()

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
        data = "%s,%s,%s" % (str(self.id), str(time.time()), reducedimg.tostring())
        #print "id sent: %s" % (str(self.id))
        self.id += 1
        #print len(zlib.compress(data, 9))
        self.distributeData(data)
        if (time.time() - self.lastSuccess)*1000 >= self.THRESHOLD:
            # publish task with higher bounty
            self.initBounty += 1
            publishTask()
        elif (time.time() - self.lastSuccess)*1000 < self.THRESHOLD and self.initBounty > self.baseBounty:
            # I wonder what this would do????
            self.initBounty -= 1
            publishTask()

    def distributeData(self, data):
        for datacenter in self.dataCenters:
            self.sock.sendto(zlib.compress(data, 9), datacenter)

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
        msg.bountyHunters = ["10.112.120.247", "104.131.172.175", "45.55.11.33"]
        msg.initialBounty = self.initBounty
        msg.bountyRate = 1
        msg.deadline = 30
        msg.inputPort = INPORT
        msg.outputPort = OUTPORT

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

