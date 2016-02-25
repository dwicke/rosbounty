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


def sendSuccess(task, taskID, winnerIP, totalTime, succCount, recvCount):
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
    msg.succCount = succCount
    msg.recvCount = recvCount
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
    #rate = rospy.Rate(10) # 10hz


    rospy.on_shutdown(shutdown)

    try:
        port = 15000


        udpCon = ConnectionManager('udp')
        udpCon.addClient('104.131.172.175', port)
        udpCon.addClient('10.112.120.247', port)
        udpCon.addClient('45.55.11.33', port)
        udpCon.addClient('10.112.120.41', port)
        udpCon.send('HI I am udp motion message')
        while len(udpCon.recv(1)) == 0:
            udpCon.send('HI I am udp motion message')
        print 'Recved something...?'
        curFor = 0.0
        curAng = 0.0
        preID = -1
        count = 0
        frequency = 5
        endFreq = 60
        startTime = 0
        succCount = 0 # this is the total number of times sent succ message
        recvCount = 0 # this is the total number of times recv vel messages
        succRate = []
        freqData = []
        while not rospy.is_shutdown() and frequency <= endFreq:
            if count == 100:
                udpCon.send('HI I am udp motion message')
                count = 0
            count += 1
            curtime = time.time() # don't want to count the wasted time of the recv...
            if curtime - startTime >= 120.0:
                startTime = curTime
                succRate.append((frequency, succCount / recvCount))
                print "frequency was: %d and the succRate was %f" % (frequency, succCount / recvCount)

                freqData.append((frequency, succCount / recvCount))

                succCount = 0 # reset
                recvCount = 0 # reset
                frequency += 5

            recvData = udpCon.recv(0.3)
            data_ar, addr = decideWinner(recvData)
            if addr != None:

                recvID = int(data_ar[2])
                recvTS = float(data_ar[3])
                taskName = data_ar[4].strip()
                forward = float(data_ar[0])
                ang = float(data_ar[1])
                totalTime = curtime - recvTS

                if recvID > preID:
                    # do motion stuff
                    if curFor != forward or curAng != ang:
                        print "forward: %d ang: %d from %s" % (forward, ang, addr)
                        robot_vel(forward, ang)
                        curFor = forward
                        curAng = ang


                    if startTime == 0:
                        startTime = curtime

                    preID = recvID
                    recvCount += 1 # i got something that i maybe can use
                    # now send the success message as long as the totalTime is less than the threshold
                    if totalTime <= (1/frequency) and frequency <= endFreq:
                        # task, taskID, winnerIP, totalTime
                        succCount += 1
                        sendSuccess(taskName, recvID, addr, totalTime, succCount, recvCount)
                    #else:
                        # condsider sending a success message with no one as the winner?...
        # end while
        # write out the data
        sendSuccess('', -1, '', 0, 0, 0)

        file = open("freqData.dat","wb")
        for item in freqData:
            file.write("%f, %f\n" % (item[0], item[1]))
        file.close()
    except socket.error, msg:
        print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        pass


