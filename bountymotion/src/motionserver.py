#!/usr/bin/env python

import socket
import rospy
import time
from geometry_msgs.msg import Twist
from bountybondsman.msg import success
from ConnectionManager import ConnectionManager


def robot_vel(forward, angular):
    twist = Twist()
    twist.linear.x = forward
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angular
    pub.publish(twist)


def sendSuccess(task, taskID, winnerIP, totalTime):
    '''
    string task
    uint32 taskID
    string winnerIP
    float64 totalTime
    '''
    msg = success()
    msg.task = task
    msg.taskID = taskID
    msg.winnerIP = winnerIP
    msg.totalTime = totalTime
    successPub.publish(msg)



def decideWinner(recvData):
    maxID = -1
    curWinner = None
    winnerIP = None
    for datum in recvData:
        if datum[0] != 'connected':
            data_ar = datum[0].split(',')
            recvID = int(data_ar[2])
            if recvID > maxID:
                maxID = recvID
                curWinner = data_ar
                winnerIP = datum[1][0]
    return curWinner, winnerIP

def shutdown():
    # stop the robot
    robot_vel(0,0)

    # close the server
    #server_socket.close()


if __name__ == "__main__":

    rospy.init_node('motionserver', anonymous=True)
    global pub
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    global successPub
    successPub = rospy.Publisher('/bountybondsman/success', success, queue_size=10)
    rate = rospy.Rate(10) # 10hz


    rospy.on_shutdown(shutdown)

    try:
        port = 15000


        udpCon = ConnectionManager('udp')
        udpCon.addClient('104.131.172.175', port)
        udpCon.addClient('10.112.120.247', port)
        udpCon.addClient('45.55.11.33', port)
        udpCon.send('HI I am udp motion message')
        while len(udpCon.recv(1)) == 0:
            udpCon.send('HI I am udp motion message')
        print 'Recved something...?'
        curFor = 0.0
        curAng = 0.0
        preID = -1
        while not rospy.is_shutdown():
            recvData = udpCon.recv(0.3)
            udpCon.send('HI I am udp motion message')
            data_ar, addr = decideWinner(recvData)
            if addr != None:
                curtime = time.time()
                recvID = int(data_ar[2])
                recvTS = float(data_ar[3])
                taskName = data_ar[4].strip()
                forward = float(data_ar[0])
                ang = float(data_ar[1])
                totalTime = curtime - recvTS

                if recvID > preID:
                    preID = recvID
                    if curFor != forward or curAng != ang:
                        print "forward: %d ang: %d from %s" % (forward, ang, addr)
                        robot_vel(forward, ang)
                        curFor = forward
                        curAng = ang
                    # now send the success message
                    # task, taskID, winnerIP, totalTime
                    sendSuccess(taskName, recvID, addr, totalTime)

    except socket.error, msg:
        print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        pass


