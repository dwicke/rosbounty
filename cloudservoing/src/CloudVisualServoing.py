
## ACH and comm stuff
import ach
import subprocess
from ctypes import *
import zlib


## ros stuff
import rospy
import time
import signal, os

## camera stuff
import numpy as np
import cv2

## messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from DataCollector import DataCollector


def shutdown():
    # stop the robot
    robot_vel(0,0)
    # write out the success and fails?


def robot_vel(forward, angular):
    twist = Twist()
    twist.linear.x = forward
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angular
    pub.publish(twist)


class TaskData(Structure):
    _fields_ = [('id', c_double),
                ('img', c_char_p)]

class VelDat(Structure):
    _fields_ = [('forwardVelocity', c_double),
                ('angularVelocity', c_double),
                ('id', c_double)]

if __name__ == "__main__":

    rospy.init_node('motionserver', anonymous=True)
    global pub
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    ## not sure yet how to do this.. i'd like to not use ROS for getting images
    subscriber = rospy.Subscriber("/camera/image_raw", Image, self.callback,  queue_size = 1)
    rospy.on_shutdown(shutdown)

    # how long do we wait for a message from the servers
    waitTime = 0.01 ## 100 hz

    f = open('ipaddresses.txt', 'r')
    servers = f.readlines()
    f.close()


    ## build list of channels for sending and receiving
    taskSendChannels = []
    taskRecvChannels = []
    for server in servers:
        taskSendChannels.append(ach.Channel(server + "-VSTaskImg")) # sending on
        taskRecvChannels.append(ach.Channel(server + "-VSResp")) # receiving from


    ## open up the video camera
    cap = cv2.VideoCapture(0)
    id = 0.0
    failCount = 0
    succCount = 0

    ### robot loop
    while not rospy.is_shutdown():

        ### get image data from camera and process it (don't use ROS just use openCV)
        ret, frame = cap.read()
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        ORANGE_MIN = np.array([5, 50, 50],np.uint8)
        ORANGE_MAX = np.array([15, 255, 255],np.uint8)
        reducedimg = cv2.inRange(hsv,ORANGE_MIN, ORANGE_MAX)



        ## first build the message to send
        taskReq = TaskData()
        taskReq.id = id
        taskReq.img = reducedimg.tostring()
        id = id + 1.0
        ### send image to bounty hunters (so will need a seperate channel to send images)
        for sendChan in taskSendChannels:
            sendChan.put(taskReq)

        # get the start time
        tock = time.time() + waitTime
        winner = None
         ### busy wait recv from each of the bounty hunters to get the control for the latest image. do that until either you receive a msg or it times-out
        while time.time() < tock:
            if winner == None:
                for recvChan in taskRecvChannels:
                    recvDat = VelDat()
                    recvChan.get(recvDat, wait=False, last=True)
                    if recvDat.id == (id - 1.0):
                        winner = recvDat

        if winner == None:
            ### if it times out restart the loop and count as a fail
            failCount += 1
        else:
            ### if we have a msg count as success and then send commands to the servos
            succCount += 1
            robot_vel(winner.forwardVelocity, winner.angularVelocity)

