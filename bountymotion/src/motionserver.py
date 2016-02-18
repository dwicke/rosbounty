#!/usr/bin/env python

import socket
import rospy
from geometry_msgs.msg import Twist

def robot_vel(forward, angular):
    twist = Twist()
    twist.linear.x = forward
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angular
    pub.publish(twist)


def shutdown():
    # stop the robot
    robot_vel(0)

    # close the server
    server_socket.close()

    




if __name__ == "__main__":
   
    rospy.init_node('motionserver', anonymous=True)
    global pub
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    
    rospy.on_shutdown(shutdown)

    try:
        port = 15000
        global server_socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_socket.bind(("0.0.0.0", port))

	curFor = 0.0
	curAng = 0.0
        while not rospy.is_shutdown():
            data, addr = server_socket.recvfrom(1024)
	    data_ar = data.split(',')
            forward = float(data_ar[0])
	    ang = float(data_ar[1])
	    if curFor != forward or curAng != ang:
        	print data    
		robot_vel(forward, ang)
            	curFor = forward
		curAng = ang
            
    except socket.error, msg:
        print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        pass
        

    

    


    


