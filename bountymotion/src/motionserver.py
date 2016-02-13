#!/usr/bin/env python

import socket
import rospy
from geometry_msgs.msg import Twist

def robot_vel(forward):
    twist = Twist()
    twist.linear.x = forward
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
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
        server_socket.bind(("localhost", port))

        while not rospy.is_shutdown():
            data, addr = server_socket.recvfrom(1024)
            data = float(data)
            robot_vel(data)
            
            
    except socket.error, msg:
        print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        pass
        

    

    


    


