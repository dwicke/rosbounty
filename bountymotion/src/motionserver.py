#!/usr/bin/env python
import numpy as np
from multiprocessing import Process, Manager
from geometry_msgs.msg import Twist
from bountybondsman.msg import success
from ConnectionManager import ConnectionManager
from sensor_msgs.msg import Image
from bountybondsman.msg import task
from DataCollector import DataCollector
import cv2
import zlib
import socket
import time
import ach
import sys
import time
import rospy
from ctypes import *


def decideWinner(recvData):
	maxID = -1
	curWinner = None
	winnerIP = None
	hzRecv = False
	if recvData == None:
		return curWinner, winnerIP
	for datum in recvData:
		if datum[0] != 'connected':
			data_ar = datum[0].split(',')
			recvID = int(data_ar[2])
				if recvID > maxID:
					maxID = recvID
					curWinner = data_ar
					winnerIP = datum[1][0]

		return curWinner, winnerIP

	def robot_vel(forward, angular):
		twist = Twist()
		twist.linear.x = forward
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = angular
		pub.publish(twist)






	# Ach files
	class cloud(Structure):
		_pack_ = 1
		_fields_ = [("image"  , c_char*230400)]



	def shutdown():
		# stop the robot
		robot_vel(0,0)

	def controlLoop(sharedImage):

		#rospy.init_node('motionserver', anonymous=True)
		global pub
		pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
		rospy.on_shutdown(shutdown)

		INPORT = 8052

		# set up the connection to the servers
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		dataCenters = [('10.112.120.247', INPORT), ('104.131.172.175', INPORT), ('159.203.67.159', INPORT), ('45.55.143.53', INPORT), ('45.55.143.47', INPORT), ('159.203.47.107', INPORT), ( '159.203.47.108',INPORT), ('159.203.47.109', INPORT), ('159.203.47.110', INPORT), ('129.174.121.166', INPORT)]


		port = 15000
		# tell the servers to send stuff here
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
		print 'Recv response'


		s = ach.Channel('image_chan', 5, 230)
		state = cloud()

		preID = 0
		curFor = -5.0
		curAng = -5.0

		totalInc = 0.0
		succInc = 0.0

		f = 5.0
		T = 1.0 / f
		HEIGHT = 240
		WIDTH = 320
		CHANNELS = 3


		for i in range(1,15):
			succInc = 0.0
			totalInc = 0.0

			doLoop = True
			fn = f*i
			T = 1.0/fn


			tick = time.time()
		#[statuss, framesizes] = s.get(state, wait=False, last=True)
		#print str(state.image)
		
			image = sharedImage[0]
			# process the image
			print len(image.tostring())

		
		totalInc += 1.0
		data = "%s,%s,%s" % (str(totalInc), str(tick), image.tostring())

		for datacenter in dataCenters:
			sock.sendto(zlib.compress(data, 3), datacenter)

		t = time.time()

		recvData = udpCon.recv(T-(t-tick))
		data_ar, addr = decideWinner(recvData)
		if addr != None:
			recvID = int(data_ar[2])
			recvTS = float(data_ar[3])
			taskName = data_ar[4].strip()
			forward = float(data_ar[0])
			ang = float(data_ar[1])
			if recvID > preID:
				preID = recvID
				if curFor != forward or curAng != ang:
					robot_vel(forward, ang)
					curFor = forward
					curAng = ang

			if recvID == totalInc: # we are getting it in time
				succInc += 1
			tock = time.time()
			if T - (tock-tick) > 0:
				time.sleep(T - (tock - tick))






class image_feature(object):

	def __init__(self, sh_image):
		'''Initialize ros subscriber'''

		self.sh_image = sh_image
		self.initBounty = 30
		## setup the publisher
		self.taskPub = rospy.Publisher('/bountybondsman/task', task, queue_size=10)

		# subscribed Topic
		self.subscriber = rospy.Subscriber("/camera/image_raw", Image, self.callback,  queue_size = 1)
		print 'Finished creating the image_features'

	def callback(self, ros_data):
		'''Callback function of subscribed topic.
		Here images get put into the shared memory'''

		self.image = bytearray(ros_data.data)
		
		HEIGHT = 240
		WIDTH = 320
		CHANNELS = 3
		image = np.array(self.image, dtype="uint8").reshape(HEIGHT,WIDTH,CHANNELS)
		#print image

		hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		ORANGE_MIN = np.array([5, 50, 50],np.uint8)
		ORANGE_MAX = np.array([15, 255, 255],np.uint8)
		reducedimg = cv2.inRange(hsv,ORANGE_MIN, ORANGE_MAX)

        	self.sh_image[0] = reducedimg
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
		INPORT = 8052
		OUTPORT = 15000
		msg = task()
		msg.taskName = "visualServoing"
		# me, nyc, sfo
		msg.bountyHunters = ["10.112.120.247", "104.131.172.175", "159.203.67.159", "45.55.143.53", "45.55.143.47", "159.203.47.107", "159.203.47.108", "159.203.47.109", "159.203.47.110", "129.174.121.166"]
		msg.initialBounty = self.initBounty
		msg.bountyRate = 1
		msg.deadline = 30
		msg.inputPort = INPORT
		msg.outputPort = OUTPORT
		print 'publishing the task'
		self.taskPub.publish(msg)



def main(args):
	'''Initializes and cleanup ros node'''
	manager = Manager()
	imageBuffer = manager.list([1])
	ic = image_feature(imageBuffer)
	rospy.init_node('bountymotion', anonymous=True)
	ic.publishTask()
	# start the child process
	
	p = Process(target=controlLoop, args=(imageBuffer))
	p.daemon = True
	p.start()


	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
	main(sys.argv)

