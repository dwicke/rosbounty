#!/usr/bin/env python

import socket
import rospy
import time
from geometry_msgs.msg import Twist
from bountybondsman.msg import success
from ConnectionManager import ConnectionManager
from DataCollector import DataCollector


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
    hzRecv = False
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
    rate = rospy.Rate(40) # 10hz


    rospy.on_shutdown(shutdown)

    try:
        port = 15000


        udpCon = ConnectionManager('udp')
        udpCon.addClient('10.112.120.247', port)
        # NY
        udpCon.addClient('104.131.172.175', port)
        udpCon.addClient('159.203.67.159', port)
        udpCon.addClient('45.55.143.47', port)
        udpCon.addClient('45.55.143.53', port)
        # TOR
        udpCon.addClient('159.203.47.110', port)
        udpCon.addClient('159.203.47.109', port)
        udpCon.addClient('159.203.47.108', port)
        udpCon.addClient('159.203.47.107', port)

        udpCon.addClient('129.174.121.166', port)


        udpCon.send('HI I am udp motion message')
        while len(udpCon.recv(1)) == 0:
            udpCon.send('HI I am udp motion message')
        print 'Recved something...?'
        curFor = 0.0
        curAng = 0.0
        preID = -1
        count = 0
        frequency = 5
        endFreq = 35
        startTime = 0.0
        succCount = 0.0 # this is the total number of times sent succ message
        recvCount = 0.0 # this is the total number of times recv vel messages
        freqData = []
        freqTS = DataCollector()
        curTS = ''
        interval = 120.0
        hzRecv = True
        while not rospy.is_shutdown() and frequency <= endFreq:
            if count == 100:
                #udpCon.send('HI I am udp motion message')
                count = 0
            count += 1

            recvData = udpCon.recv(0.005)
            curtime = time.time()



            data_ar, addr = decideWinner(recvData)


            if addr != None:

                if curtime - startTime >= interval:
                    startTime = curtime
                    if frequency != 5:
                        hzRecv = False
                        if recvCount == 0.0:
                            freqData.append((frequency, 0.0))
                            print "frequency was: %d and the succRate was 0" % (frequency)
                        else:
                            freqData.append((frequency, succCount / recvCount))
                            print "frequency was: %d and the succRate was %f" % (frequency, succCount / recvCount)
                    succCount = 0.0 # reset
                    recvCount = 0.0 # reset
                    frequency += 5
                    curTS = 'tsData_' + str(frequency)
                    print 'frequency = %d' % (frequency)


                recvID = int(data_ar[2])
                recvTS = float(data_ar[3])
                taskName = data_ar[4].strip()
                forward = float(data_ar[0])
                ang = float(data_ar[1])
                totalTime = curtime - recvTS
                #print 'CurTime = %f and recvTS = %f and totalTime = %f' % (curtime, recvTS, totalTime)

                if recvID > preID:

                    print "recvID = %d %f - %f = %f" %(recvID, curtime, recvTS, totalTime)
                    #print "recvID = %d forward: %f ang: %f from %s total time %f desired time = %f" % (recvID, forward, ang, addr, totalTime, 1.0/frequency)
                    # do motion stuff
                    if curFor != forward or curAng != ang:
                        robot_vel(forward, ang)
                        curFor = forward
                        curAng = ang

                    preID = recvID
                    recvCount += 1.0 # i got something that i maybe can use
                    # now send the success message as long as the totalTime is less than the threshold
                    if totalTime <= (1.0/frequency) and frequency <= endFreq:
                        # task, taskID, winnerIP, totalTime
                        succCount += 1.0
                        #sendSuccess(taskName, recvID, addr, totalTime, succCount, recvCount)

                    #freqTS.addPoint(curTS, (recvTS, succCount / recvCount))

                    #else:
                        # condsider sending a success message with no one as the winner?...
        # end while
        # write out the data
        sendSuccess('visualServoing', -1, '', 0, 0, 0) # should be more general...
        freqTS.writeData()

        file = open("/home/pi/freqData.dat","w")
        for item in freqData:
            file.write("%f, %f\n" % (item[0], item[1]))
        file.close()
    except socket.error, msg:
        print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        pass


