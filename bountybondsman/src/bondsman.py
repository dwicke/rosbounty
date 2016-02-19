#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np

# Ros libraries
import roslib
import rospy

import socket
from bountybondsman.msg import task

PORT = 14000

# Ros Messages

VERBOSE=False

class bondsman:

    def __init__(self):
        '''Initialize ros subscriber'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.taskList = {}

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/bountybondsman/task",
            task, self.taskCallback,  queue_size = 1)

        if VERBOSE :
            print "subscribed to /bountybondsman/task"


    def successCallback(self, ros_data):
        '''Callback function of subscribed topic.
        Here success messages get sent to the bounty hunters'''
        for bountyHunter in self.taskList[ros_data.task].bountyHunters:
            self.sock.sendto(ros_data, (bountyHunter, PORT))

    def taskCallback(self, ros_data):
        '''Callback function of subscribed topic.
	    string taskName
            string[] bountyHunters
            float64 initialBounty
            float64 bountyRate
            float64 deadline
            uint32 inputPort
            uint32 outputPort
        Here tasks that get forwarded on to bounty hunters'''
	msgdata = [ros_data.taskName, ros_data.bountyHunters, ros_data.initialBounty, ros_data.bountyRate, ros_data.deadline, ros_data.inputPort, ros_data.outputPort]
	print str(msgdata)
        self.taskList[ros_data.taskName] = ros_data
        for bountyHunter in ros_data.bountyHunters:
            self.sock.sendto(str(msgdata), (bountyHunter, PORT))


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

