#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np

# Ros libraries
import roslib
import rospy
import json
import socket
from bountybondsman.msg import task
from bountybondsman.msg import success

PORT = 14000

# Ros Messages

VERBOSE=False

class bondsman:

    def __init__(self):
        '''Initialize ros subscriber'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.taskList = {}
        self.taskMsg = {}

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/bountybondsman/task",
            task, self.taskCallback,  queue_size = 1)
        self.subscriber = rospy.Subscriber("/bountybondsman/success",
            success, self.successCallback,  queue_size = 1)

        rospy.Timer(rospy.Duration(10), self.taskResend)
        if VERBOSE :
            print "subscribed to /bountybondsman/task"


    def taskResend(self, event):
        # http://library.isr.ist.utl.pt/docs/roswiki/rospy(2f)Overview(2f)Time.html
        print 'Sending the task messages again!'
        for task in self.taskMsg.keys():
            for bountyHunter in self.taskList[task].bountyHunters:
                print json.dumps(self.taskMsg[task])
                self.sock.sendto(json.dumps(self.taskMsg[task]), (bountyHunter, PORT))
        print 'finished sending'

    def successCallback(self, ros_data):
        '''Callback function of subscribed topic.
        Here success messages get sent to the bounty hunters
        string task
        uint32 taskID
        string winnerIP
        float64 totalTime
        '''
        msgdata = ['success', ros_data.task, ros_data.taskID, ros_data.winnerIP, ros_data.totalTime, ros_data.succCount, ros_data.recvCount]
        for bountyHunter in self.taskList[ros_data.task].bountyHunters:
            self.sock.sendto(json.dumps(msgdata), (bountyHunter, PORT))
        if ros_data.taskID == -1:
            ## then I'm done!
            rospy.signal_shutdown("received succ with -1 id")

    def taskCallback(self, ros_data):
        '''Callback function of subscribed topic
	    string taskName
            string[] bountyHunters
            float64 initialBounty
            float64 bountyRate
            float64 deadline
            uint32 inputPort
            uint32 outputPort
        Here tasks that get forwarded on to bounty hunters'''
        msgdata = ['task', ros_data.taskName, ros_data.bountyHunters, ros_data.initialBounty, ros_data.bountyRate, ros_data.deadline, ros_data.inputPort, ros_data.outputPort]
        print str(json.dumps(msgdata))
        self.taskList[ros_data.taskName] = ros_data
        self.taskMsg[ros_data.taskName] = msgdata
        for bountyHunter in ros_data.bountyHunters:
            self.sock.sendto(json.dumps(msgdata), (bountyHunter, PORT))


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('bondsman', anonymous=True)
    myBondsman = bondsman()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)

