

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>



using geometry_msgs::Twist;
using namespace std;

ros::Publisher vel_topic;
ros::Duration duration;

// data structure for set the velocities
Twist vel;




void stop_robot()
{
  vel.linear.x = 0.0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;

  vel_topic.publish(vel);
  
}

void quit(int sig)
{
  stop_robot();
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{ 
  // the third argument is the name of the node
  ros::init(argc, argv, "teleport");

  // NodeHandle is the main access point to communication with the ROS system
  ros::NodeHandle nh;

  // The advertise() function is how you tell ROS that you want to publish on a given topic name.
  // The second parameter is the size of the message queue used for publishing messages
  vel_topic = nh.advertise<Twist>("/RosAria/cmd_vel", 1000);  
  
  // call quit function is C-c is pressed
  signal(SIGINT,quit);

  ros::Rate r(10); //10 hz rate

  
  // 5 seconds
  duration = ros::Duration(5);
 
 

  vel.linear.x = 0.2;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 1.8;
 


  while(ros::ok())
    {
      vel_topic.publish(vel);
      ROS_INFO("send");
      duration.sleep();
    }
  
  return(0);
}
